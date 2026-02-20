from __future__ import annotations

from pathlib import Path
from typing import Dict, Optional

import polars as pl


class TSM:
    """Traffic State Monitoring data.

    Builds density/flow clusters from per-vehicle detector data stored in a
    Polars ``DataFrame``.

    Parameters
    ----------
    data : pl.DataFrame
        Raw detector data.  Must contain at least the columns
        ``detector``, ``timestamp``, and ``speed_kph`` (or names that are
        mapped to them via *column_mapping*).
    column_mapping : dict, optional
        ``{source_name: english_name}`` mapping used to rename columns
        before processing.  If ``None`` the DataFrame is used as-is and
        must already contain the expected English column names.

        Required target columns:

        - ``detector``: unique ID of the traffic detector (e.g. loop sensor).
        - ``timestamp``: timestamp of the vehicle passage (must be a
          Polars datetime type).
        - ``speed_kph``: speed of the vehicle in km/h.

        Optional target columns:

        - ``direction``: direction of travel (e.g. 'N', 'S', etc.).
          If present, clusters are computed separately for each direction.
        - ``lane``: lane number.  If present, density and flow are
          normalised by the number of lanes in each cluster.
    """

    def __init__(
        self,
        data: pl.DataFrame,
        column_mapping: Optional[Dict[str, str]] = None,
    ) -> None:
        if column_mapping is not None:
            rename = {
                src: eng for src, eng in column_mapping.items() if src in data.columns
            }
            self._df: pl.DataFrame = data.rename(rename)
        else:
            self._df = data

        # Detect optional columns
        self._has_direction = "direction" in self._df.columns
        self._has_lane = "lane" in self._df.columns

        # Validate required columns
        for required in ("detector", "timestamp", "speed_kph"):
            if required not in self._df.columns:
                raise ValueError(
                    f"Column '{required}' not found in the DataFrame. "
                    f"Available columns: {self._df.columns}"
                )

        self._result: Optional[pl.DataFrame] = None

    # ------------------------------------------------------------------
    # helpers
    # ------------------------------------------------------------------
    @property
    def _group_cols(self) -> list[str]:
        """Grouping columns depending on available optional fields."""
        cols = ["detector"]
        if self._has_direction:
            cols.append("direction")
        return cols

    # ------------------------------------------------------------------
    # public API
    # ------------------------------------------------------------------
    def clusterize(
        self,
        min_vehicles: int = 5,
        gap_factor: float = 3.0,
    ) -> "TSM":
        """Run the clustering pipeline.

        Parameters
        ----------
        min_vehicles : int
            Minimum number of vehicles for a cluster to be kept.
        gap_factor : float
            A new cluster starts when the estimated distance exceeds
            ``gap_factor * (speed / 3.6)`` metres.

        Returns
        -------
        TSM
            ``self``, so calls can be chained.
        """
        group = self._group_cols

        # --- lanes sub-table (only when lane info is available) -----------
        lanes_df: Optional[pl.DataFrame] = None
        if self._has_lane:
            lanes_df = self._df.group_by(group).agg(
                pl.col("lane").n_unique().alias("n_lanes")
            )

        # --- main pipeline ------------------------------------------------
        result = (
            self._df.sort(group + ["timestamp"])
            .with_columns(
                (pl.col("timestamp") - pl.col("timestamp").shift(1))
                .dt.total_seconds()
                .over(group)
                .alias("delta_t_s")
            )
            .with_columns(
                (pl.col("speed_kph") * pl.col("delta_t_s") / 3.6).alias("distance_m")
            )
            .with_row_index("row_idx")
            .with_columns(
                (
                    (pl.col("distance_m") > gap_factor * (pl.col("speed_kph") / 3.6))
                    | pl.col("delta_t_s").is_null()
                ).alias("new_cluster")
            )
            .with_columns(
                pl.col("new_cluster")
                .cast(pl.Int32)
                .cum_sum()
                .over(group)
                .alias("cluster_local_id")
            )
            .group_by(group + ["cluster_local_id"])
            .agg(
                pl.col("speed_kph").mean().alias("mean_speed_kph"),
                pl.len().alias("num_vehicles"),
                (pl.col("distance_m") * 1e-3).sum().alias("cluster_len_km"),
                pl.col("delta_t_s").sum().alias("cluster_dt_s"),
            )
            .filter(pl.col("num_vehicles") > min_vehicles)
        )

        # --- join lane count & compute density / flow ---------------------
        if lanes_df is not None:
            result = result.join(lanes_df, on=group, how="left").with_columns(
                (
                    pl.col("num_vehicles")
                    / pl.col("cluster_len_km")
                    / pl.col("n_lanes")
                ).alias("density"),
                (
                    pl.col("num_vehicles")
                    * 3.6e3
                    / pl.col("cluster_dt_s")
                    / pl.col("n_lanes")
                ).alias("flow"),
            )
        else:
            # Without lane info assume 1 lane
            result = result.with_columns(
                (pl.col("num_vehicles") / pl.col("cluster_len_km")).alias("density"),
                (pl.col("num_vehicles") * 3.6e3 / pl.col("cluster_dt_s")).alias("flow"),
            )

        self._result = result.sort(group + ["cluster_local_id"])
        return self

    # ------------------------------------------------------------------
    # accessors
    # ------------------------------------------------------------------
    @property
    def result(self) -> pl.DataFrame:
        """Return the clustered result DataFrame.

        Raises
        ------
        RuntimeError
            If :meth:`clusterize` has not been called yet.
        """
        if self._result is None:
            raise RuntimeError("Call .clusterize() before accessing .result")
        return self._result

    @property
    def df(self) -> pl.DataFrame:
        """Alias for :attr:`result`."""
        return self.result

    def to_csv(self, path: str | Path, **kwargs) -> None:
        """Write the result to a CSV file."""
        self.result.write_csv(path, **kwargs)

    def to_parquet(self, path: str | Path, **kwargs) -> None:
        """Write the result to a Parquet file."""
        self.result.write_parquet(path, **kwargs)

    def __repr__(self) -> str:
        status = "clusterized" if self._result is not None else "raw"
        rows = len(self._result) if self._result is not None else len(self._df)
        return f"TSM(status={status}, rows={rows})"
