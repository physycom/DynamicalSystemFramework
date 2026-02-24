from __future__ import annotations

from pathlib import Path
from typing import Dict, Optional

from pyspark.sql import DataFrame, SparkSession, Window
import pyspark.sql.functions as F
import pyspark.sql.types as T


def _get_or_create_spark() -> SparkSession:
    """Return the active SparkSession or create a local one."""
    return (
        SparkSession.builder
        .master("local[*]")
        .appName("TSM")
        .config("spark.driver.memory", "128g") \
        .config("spark.executor.memory", "128g") \
        .config("spark.sql.shuffle.partitions", "1600") \
        .config("spark.sql.adaptive.enabled", "true") \
        .config("spark.sql.adaptive.coalescePartitions.enabled", "true") \
        .getOrCreate()
    )


class TSM:
    """Traffic State Monitoring data.

    Builds density/flow clusters from per-vehicle detector data stored in a
    PySpark ``DataFrame``.

    Parameters
    ----------
    data : pyspark.sql.DataFrame
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
          PySpark ``TimestampType``).
        - ``speed_kph``: speed of the vehicle in km/h.

        Optional target columns:

        - ``direction``: direction of travel (e.g. 'N', 'S', etc.).
          If present, clusters are computed separately for each direction.
        - ``lane``: lane number.  If present, density and flow are
          normalised by the number of lanes in each cluster.
    """

    def __init__(
        self,
        data: DataFrame,
        column_mapping: Optional[Dict[str, str]] = None,
    ) -> None:
        if column_mapping is not None:
            df = data
            for src, eng in column_mapping.items():
                if src in df.columns:
                    df = df.withColumnRenamed(src, eng)
            self._df: DataFrame = df
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

        self._result: Optional[DataFrame] = None

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
        lanes_df: Optional[DataFrame] = None
        if self._has_lane:
            lanes_df = self._df.groupBy(group).agg(
                F.count_distinct("lane").alias("n_lanes")
            )

        # --- window for per-detector and direction in ordered operations ----------------------
        w = Window.partitionBy(group).orderBy("timestamp")

        # --- main pipeline ------------------------------------------------
        df = self._df

        # delta_t_s: seconds since previous row for the same detector and direction
        df = df.withColumn(
            "prev_timestamp", F.lag("timestamp").over(w)
        ).withColumn(
            "delta_t_s",
            F.when(
                F.col("prev_timestamp").isNotNull(),
                F.col("timestamp").cast("long") - F.col("prev_timestamp").cast("long"),
            ),
        ).drop("prev_timestamp")

        # distance_m
        df = df.withColumn(
            "distance_m",
            F.col("speed_kph") * F.col("delta_t_s") / 3.6,
        )

        # new_cluster flag
        df = df.withColumn(
            "new_cluster",
            (
                (F.col("distance_m") > F.lit(gap_factor) * (F.col("speed_kph") / 3.6))
                | F.col("delta_t_s").isNull()
            ).cast("int"),
        )

        # cluster_local_id: cumulative sum of new_cluster within each detector and direction
        w_unbounded = (
            Window.partitionBy(group)
            .orderBy("timestamp")
            .rowsBetween(Window.unboundedPreceding, Window.currentRow)
        )
        df = df.withColumn(
            "cluster_local_id",
            F.sum("new_cluster").over(w_unbounded),
        )

        # aggregate per cluster
        result = (
            df.groupBy(group + ["cluster_local_id"])
            .agg(
                F.mean("speed_kph").alias("mean_speed_kph"),
                F.count("*").alias("num_vehicles"),
                F.sum(F.col("distance_m") * 1e-3).alias("cluster_len_km"),
                F.sum("delta_t_s").alias("cluster_dt_s"),
            )
            .filter(F.col("num_vehicles") > min_vehicles)
        )

        # --- join lane count & compute density / flow ---------------------
        if lanes_df is not None:
            result = result.join(lanes_df, on=group, how="left").withColumn(
                "density",
                F.col("num_vehicles") / F.col("cluster_len_km") / F.col("n_lanes"),
            ).withColumn(
                "flow",
                F.col("num_vehicles") * 3.6e3 / F.col("cluster_dt_s") / F.col("n_lanes"),
            )
        else:
            # Without lane info assume 1 lane
            result = result.withColumn(
                "density",
                F.col("num_vehicles") / F.col("cluster_len_km"),
            ).withColumn(
                "flow",
                F.col("num_vehicles") * 3.6e3 / F.col("cluster_dt_s"),
            )

        self._result = result.orderBy(*group, "cluster_local_id")
        return self

    # ------------------------------------------------------------------
    # accessors
    # ------------------------------------------------------------------
    @property
    def result(self) -> DataFrame:
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
    def df(self) -> DataFrame:
        """Alias for :attr:`result`."""
        return self.result

    def to_csv(self, path: str | Path, **kwargs) -> None:
        """Write the result to a CSV directory (Spark partitioned output).

        Parameters
        ----------
        path : str or Path
            Destination directory.  Spark writes one or more part-* files.
        **kwargs
            Extra options forwarded to ``DataFrameWriter.csv()``.
        """
        self.result.write.option("header", "true").csv(str(path), **kwargs)

    def to_parquet(self, path: str | Path, **kwargs) -> None:
        """Write the result to a Parquet directory (Spark partitioned output).

        Parameters
        ----------
        path : str or Path
            Destination directory.
        **kwargs
            Extra options forwarded to ``DataFrameWriter.parquet()``.
        """
        self.result.write.parquet(str(path), **kwargs)

    def __repr__(self) -> str:
        status = "clusterized" if self._result is not None else "raw"
        if self._result is not None:
            rows = self._result.count()
        else:
            rows = self._df.count()
        return f"TSM(status={status}, rows={rows})"
