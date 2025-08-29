"""
This script generates various plots related to traffic flow analysis, including:

1. Giant Component plot: Plots the number of components and the size of the first and second ones.
2. Fundamental diagram plot: Plots the fundamental diagram of traffic flow vs. density.
3. Fluctuations plot: Plots the fluctuations in traffic density.
4. Signal plot: Plot the signal and the peak detected by the peak detection algorithm.

Global Variables:
- Y_MAX (int): Maximum y-axis value for the Giant Component plot.
- MU_TIME (str): Unit of time.
- MU_SPEED (str): Unit of speed.
- MU_DENSITY (str): Unit of density.
- MU_FLOW (str): Unit of flow.

Functions:
- adjust_dataframe(df): Adjusts the dataframe for analysis.

Plots:
1. Giant Component plot:
    - Plots the number of components and the size of the first and second giant components.
    - Uses a secondary y-axis for the giant component sizes.
    - Saves the plot if SAVE_FIG is True.

2. Fundamental diagram plot:
    - Plots the fundamental diagram of traffic flow vs. density.
    - Adds a vertical line and shaded area to indicate a specific density range.
    - Adds an inset plot for a zoomed-in view of a specific region.
    - Saves the plot if SAVE_FIG is True.

2.1 Fundamental diagram plot (optimized):
    - Similar to the fundamental diagram plot, including error bars for normal and optimized data.
    - Adds an inset plot for a zoomed-in view of a specific region.
    - Saves the plot if SAVE_FIG is True.

3. Fluctuations plot:
    - Plots the fluctuations in traffic density over time.

3.1 Fluctuations plot (optimized):
    - Similar to the fluctuations plot, but includes error bars for normal and optimized data.
    - Adds an inset plot for a zoomed-in view of a specific region.
    - Saves the plot if SAVE_FIG is True.

4. Signal plot:
    - Compares mean density and flow fluctuations over time.
    - Highlights significant peaks and time intervals.
    - Uses dual y-axes for density and flow fluctuations.
    - Saves the plot if SAVE_FIG is True.
"""

import os
import sys
from argparse import ArgumentParser
import pandas as pd
import seaborn as sns
from matplotlib import pyplot as plt
import numpy as np
import networkx as nx
from tqdm import tqdm

PRESENTATION = False


class RealTimePeakDetection:
    """
    Source:
        https://stackoverflow.com/questions/22583391/peak-signal-detection-in-realtime-timeseries-data
    """

    def __init__(self, array, lag, threshold, influence):
        self.y = list(array)
        self._params = {"lag": lag, "threshold": threshold, "influence": influence}
        self.signals = [0] * len(self.y)
        self.filtered_y = np.array(self.y).tolist()
        self.avg_filter = [0] * len(self.y)
        self.std_filter = [0] * len(self.y)
        self.avg_filter[self._params["lag"] - 1] = np.mean(
            self.y[0 : self._params["lag"]]
        ).tolist()
        self.std_filter[self._params["lag"] - 1] = np.std(
            self.y[0 : self._params["lag"]]
        ).tolist()

    def thresholding_algo(self, new_value):
        """
        Use thresholding algorithm
        """
        self.y.append(new_value)
        length = len(self.y) - 1
        if length < self._params["lag"]:
            return 0
        if length == self._params["lag"]:
            self.signals = [0] * len(self.y)
            self.filtered_y = np.array(self.y).tolist()
            self.avg_filter = [0] * len(self.y)
            self.std_filter = [0] * len(self.y)
            self.avg_filter[self._params["lag"]] = np.mean(
                self.y[0 : self._params["lag"]]
            ).tolist()
            self.std_filter[self._params["lag"]] = np.std(
                self.y[0 : self._params["lag"]]
            ).tolist()
            return 0

        self.signals += [0]
        self.filtered_y += [0]
        self.avg_filter += [0]
        self.std_filter += [0]

        if abs(self.y[length] - self.avg_filter[length - 1]) > (
            self._params["threshold"] * self.std_filter[length - 1]
        ):
            if self.y[length] > self.avg_filter[length - 1]:
                self.signals[length] = 1
            else:
                self.signals[length] = -1

            self.filtered_y[length] = (
                self._params["influence"] * self.y[length]
                + (1 - self._params["influence"]) * self.filtered_y[length - 1]
            )
            self.avg_filter[length] = np.mean(
                self.filtered_y[(length - self._params["lag"]) : length]
            )
            self.std_filter[length] = np.std(
                self.filtered_y[(length - self._params["lag"]) : length]
            )
        else:
            self.signals[length] = 0
            self.filtered_y[length] = self.y[length]
            self.avg_filter[length] = np.mean(
                self.filtered_y[(length - self._params["lag"]) : length]
            )
            self.std_filter[length] = np.std(
                self.filtered_y[(length - self._params["lag"]) : length]
            )

        return self.signals[length]

    @property
    def params(self):
        """Return algorithm parameters"""
        return self._params


def adjust_dataframe(_df):
    """
    Adjusts measurement units for the analysis.
    """
    # scale the time to hours
    _df["time"] = _df["time"] / 3600
    _df["mean_traveltime"] = _df["mean_traveltime"] / 3600
    _df["mean_traveltime_err"] = _df["mean_traveltime_err"] / 3600
    # scale speed to km/h
    _df["mean_speed"] = _df["mean_speed"] * 3.6
    _df["mean_speed_err"] = _df["mean_speed_err"] * 3.6
    # scale density to veh/km
    _df["mean_density"] = _df["mean_density"] * 1000
    _df["mean_density_err"] = _df["mean_density_err"] * 1000
    # scale flow to veh/h
    _df["mean_flow"] = _df["mean_flow"] * 3600
    _df["mean_flow_err"] = _df["mean_flow_err"] * 3600
    _df["mean_flow_spires"] = _df["mean_flow_spires"] * 3600
    _df["mean_flow_spires_err"] = _df["mean_flow_spires_err"] * 3600

    # normalizing fluctuations...
    _df["norm_density_fluctuations"] = _df["mean_density_err"] / _df["mean_density"]
    _df["norm_flow_fluctuations"] = _df["mean_flow_err"]  # / df["mean_flow"]
    _df["norm_speed_fluctuations"] = _df["mean_speed_err"]  # / df["mean_speed"]
    _df["norm_flow_spires_fluctuations"] = _df[
        "mean_flow_spires_err"
    ]  # / df["mean_flow_spires"]

    return _df


if __name__ == "__main__":
    parser = ArgumentParser(
        description="Script to generate various traffic flow analysis plots."
    )
    parser.add_argument(
        "--input_folder",
        type=str,
        required=True,
        help="Path to the first input folder.",
    )
    parser.add_argument(
        "--input_folder_opt",
        type=str,
        required=False,
        help="Path to the second input folder (optional).",
        default=None,
    )
    args = parser.parse_args()

    INPUT_FOLDER = args.input_folder
    INPUT_FOLDER_OPT = args.input_folder_opt

    ############################################
    # Constant definition
    ############################################
    ALPHA = 0.95
    STREET_LENGTH = 2  # km
    STREET_CAPACITY = 225  # veh

    K_MAX = STREET_CAPACITY / STREET_LENGTH  # veh/km
    V_MAX = 50  # km/h

    K_C = K_MAX / (2 * ALPHA)
    Q_MAX = K_C * V_MAX * (1 - ALPHA * K_C / K_MAX)

    TIME_GRANULARITY = 300

    print(f"K_C = {K_C:.1f} veh/km")
    print(f"Q_MAX = {Q_MAX:.1f} veh/h")

    sns.set_style("whitegrid", {"grid.linestyle": "--"})

    MU_SPEED = "m/s"
    MU_FLOW = "veh/s"
    MU_DENSITY = "veh/m"
    MU_TIME = "s"

    LABEL_DENSITY = "\\rho"
    LABEL_FLOW = "\\phi"
    LABEL_SPEED = "v"

    OUT_IMG_FOLDER = "./images/"
    SAVE_FIG = True

    if not os.path.exists(OUT_IMG_FOLDER):
        os.makedirs(OUT_IMG_FOLDER)
    else:
        # clear folder
        for f in os.listdir(OUT_IMG_FOLDER):
            os.remove(os.path.join(OUT_IMG_FOLDER, f))

    ############################################
    # Load data
    ############################################

    adj = np.loadtxt("../examples/data/matrix.dat", skiprows=1)
    G = nx.from_numpy_array(adj, create_using=nx.DiGraph())
    # read the coordinates
    coord = pd.read_csv("../examples/data/coordinates.csv", sep=";")
    coord = coord.set_index("nodeId")
    # create the pos
    pos = {}
    # coord has id as index with lat, lon columns
    for node in G.nodes():
        pos[node] = (coord.loc[node]["lon"], coord.loc[node]["lat"])

    # Draws the graph for debugging
    nx.draw(G, pos, with_labels=True, node_size=100, node_color="skyblue", font_size=8)
    plt.show()

    df_array = []
    df_den_array = []

    # for every folder in INPUT_FOLDER load data.csv
    for folder in os.listdir(INPUT_FOLDER):
        if os.path.isdir(os.path.join(INPUT_FOLDER, folder)):
            df_array.append(
                pd.read_csv(INPUT_FOLDER + "/" + folder + "/" + "data.csv", sep=";")
            )
            # clear temp
            temp = pd.read_csv(
                INPUT_FOLDER + "/" + folder + "/" + "densities.csv", sep=";"
            )
            temp = temp.set_index("time")
            temp = temp * 1000

            # take only rows with index % 300 == 0
            temp = temp[temp.index % TIME_GRANULARITY == 0]
            df_den_array.append(temp)

    # make an unique dataframe as mean of all the dataframes
    DF = None
    if len(df_array) > 0:
        DF = pd.concat(df_array)
        DF = DF.groupby(DF.index).mean()
    else:
        print("No data found")
        sys.exit()

    DF_DEN = None
    if len(df_den_array) > 0:
        DF_DEN = pd.concat(df_den_array)
        DF_DEN = DF_DEN.groupby(DF_DEN.index).mean()
    else:
        print("No density data found")
        sys.exit()

    ############################################
    # Load densities
    ############################################

    # compute mean density for each row
    mean_density = DF_DEN.mean(axis=1)

    k1_array = []
    tk1_array = []

    for temp_df in tqdm(df_den_array):
        # compute mean density for each row
        temp_mean_density = temp_df.mean(axis=1)
        gc_data = []
        for time in tqdm(temp_df.index, leave=False):
            copy = G.copy()
            for col in temp_df.columns:
                index = int(col)
                density = temp_df.loc[time][col]
                src = index // G.number_of_nodes()
                dst = index % G.number_of_nodes()
                # remove edge
                if density < K_C:
                    if copy.has_edge(src, dst):
                        copy.remove_edge(src, dst)
            # print size of giant component of copy
            components = [len(c) for c in nx.weakly_connected_components(copy)]
            gc = max(components)
            second_gc = sorted(components)[-2] if len(components) > 1 else 0
            gc_data.append(
                [time / 3600, len(components), gc, second_gc, temp_mean_density[time]]
            )

        # find when the second giant component drop its size
        # make an array with the differences
        gc_data = np.array(gc_data)
        diff = np.diff(gc_data[:, 3])
        diff = np.abs(diff)
        # find the index where the difference is min
        min_diff_index = np.argmax(diff)

        k1_array.append(gc_data[min_diff_index, 4])
        tk1_array.append(gc_data[min_diff_index, 0])

    TK1 = (np.mean(tk1_array), np.std(tk1_array))
    K1 = (np.mean(k1_array), np.std(k1_array))

    print(f"TK1 = {TK1[0]:.1f} h $\\pm$ {TK1[1]:.1f} h")
    print(f"K1 = {K1[0]:.1f} veh/km $\\pm$ {K1[1]:.1f} veh/km")

    gc_data = []
    for time in tqdm(DF_DEN.index):
        copy = G.copy()
        for col in DF_DEN.columns:
            index = int(col)
            density = DF_DEN.loc[time][col]
            src = index // G.number_of_nodes()
            dst = index % G.number_of_nodes()
            # remove edge
            if density < K_C:
                if copy.has_edge(src, dst):
                    copy.remove_edge(src, dst)
        # print size of giant component of copy
        components = [len(c) for c in nx.weakly_connected_components(copy)]
        gc = max(components)
        second_gc = sorted(components)[-2] if len(components) > 1 else 0
        gc_data.append(
            [time / 3600, len(components), gc, second_gc, mean_density[time]]
        )

    # find when the second giant component drop its size
    # make an array with the differences
    gc_data = np.array(gc_data)
    diff = np.diff(gc_data[:, 3])
    # find the index where the difference is min
    min_diff_index = np.argmin(diff)

    print(f"TK1 = {gc_data[min_diff_index, 0]:.1f} h")
    print(f"K1 = {gc_data[min_diff_index, 4]:.1f} veh/km")

    ############################################
    # 1. Giant Component plot
    ############################################

    Y_MAX = 145
    # plt.figure(figsize=(8,6))
    plt.plot(gc_data[:, 4], gc_data[:, 1], label="Number of components")
    plt.ylim(0, Y_MAX)
    plt.grid(linestyle="--")
    plt.xlabel("$\\rho \\ \\left(veh/h\\right)$")
    plt.ylabel("Number of components")
    plt.title("Number of components vs. Giant Component size")

    plt.legend(loc="upper left")
    # plot also giant component size
    plt.twinx()
    if PRESENTATION:
        plt.xlim(0, 62)  # 62 or 50 for rb
    plt.ylim(0, Y_MAX)
    plt.plot(gc_data[:, 4], gc_data[:, 2], color="green", label="First Giant Component")
    plt.plot(
        gc_data[:, 4], gc_data[:, 3], color="orange", label="Second Giant Component"
    )
    plt.ylabel("Size")
    plt.legend(loc="upper right")
    # increase graph height to show the legend
    if SAVE_FIG:
        plt.savefig(OUT_IMG_FOLDER + "components.pgf")
    plt.show()

    ############################################
    # Data scaling
    ############################################

    DF = adjust_dataframe(DF)

    for temp_df in df_array:
        temp_df = adjust_dataframe(temp_df)

    MU_TIME = "h"
    MU_SPEED = "km/h"
    MU_DENSITY = "veh/km"
    MU_FLOW = "veh/h"

    ############################################
    # 2. Fundamental diagram plot
    ############################################

    fig, ax = plt.subplots()
    sns.scatterplot(
        x="mean_density",
        y="mean_flow_spires",
        data=DF,
        hue="time",
        palette="viridis",
        ax=ax,
    )

    if PRESENTATION:
        legend1 = ax.legend(title=f"Time $({MU_TIME})$", loc="lower right")
    else:
        legend1 = ax.legend(title=f"Time $({MU_TIME})$", loc="upper right")
    ax.axvspan(K1[0] - K1[1], K1[0] + K1[1], color="lightgrey", alpha=0.35)
    line = ax.axvline(
        K1[0],
        color="black",
        linestyle="--",
        label=f"$\\rho_1 = \\left({K1[0]:.1f} \\pm {K1[1]:.1f}\\right) \\ veh/km$",
    )
    # add legend for the vertical line
    leg = ax.legend(
        [line],
        [f"$\\rho_1 = \\left({K1[0]:.1f} \\pm {K1[1]:.1f}\\right) \\ veh/km$"],
        loc="upper left",
    )
    ax.add_artist(legend1)

    xlim = plt.xlim()
    if PRESENTATION:
        plt.xlim(0, 100)
    else:
        plt.xlim(0, xlim[1] * 1.1)
    ylim = plt.ylim()
    plt.ylim(0, ylim[1] * 1.1)

    ##### TL 05
    # x1, x2, y1, y2 = 15, 32, 550, 650
    # axins = ax.inset_axes(
    #   [0.15, 0.01, 0.4, 0.4], xlim=(x1, x2), ylim=(y1, y2), xticklabels=[], yticklabels=[]
    # )
    ##### TL 03
    x1, x2, y1, y2 = 13, 61, 550, 730
    axins = ax.inset_axes(
        [0.15, 0.2, 0.6, 0.4],
        xlim=(x1, x2),
        ylim=(y1, y2),
        xticklabels=[],
        yticklabels=[],
    )
    axins.axvspan(K1[0] - K1[1], K1[0] + K1[1], color="lightgrey", alpha=0.35)
    axins.axvline(
        K1[0],
        color="black",
        linestyle="--",
        label=f"$\\rho_1 = \\left({K1[0]:.1f} \\pm {K1[1]:.1f}\\right) \\ veh/km$",
    )
    ##### RB 05
    # x1, x2, y1, y2 = 12, 32, 550, 650
    # axins = ax.inset_axes(
    # [0.2, 0.01, 0.5, 0.4], xlim=(x1, x2), ylim=(y1, y2), xticklabels=[], yticklabels=[]
    # )
    sns.scatterplot(
        x="mean_density",
        y="mean_flow_spires",
        data=DF,
        hue="time",
        palette="viridis",
        ax=axins,
    )
    axins.set_xlabel("")
    axins.set_ylabel("")
    axins.legend([], [], frameon=False)  # Remove legend from axins
    axins.patch.set_edgecolor("black")
    axins.patch.set_linewidth(1)

    x_ticks_main = np.array([int(label.get_text()) for label in ax.get_xticklabels()])
    y_ticks_main = np.array([int(label.get_text()) for label in ax.get_yticklabels()])

    # Adjust the tick locations to fit within the limits of the inset axes
    x_ticks_ins = x_ticks_main[(x_ticks_main >= x1) & (x_ticks_main <= x2)]
    y_ticks_ins = y_ticks_main[(y_ticks_main >= y1) & (y_ticks_main <= y2)]

    # Set the tick locations for both axes
    axins.set_xticks(x_ticks_ins)
    axins.set_yticks(y_ticks_ins)

    axins.grid(True)
    ax.indicate_inset_zoom(axins, edgecolor="black")

    ax.set_xlabel(f"${LABEL_DENSITY} \\ \\left({MU_DENSITY}\\right)$")
    ax.set_ylabel(f"${LABEL_FLOW} \\ \\left({MU_FLOW}\\right)$")
    ax.set_title(f"Fundamental ${LABEL_FLOW}-{LABEL_DENSITY}$ diagram")

    if SAVE_FIG:
        plt.savefig(OUT_IMG_FOLDER + "fundamental_qk.pgf")
    plt.show()

    ############################################
    # 2.1 Fundamental diagram plot - if exists opt
    ############################################

    DF_OPT = None

    if INPUT_FOLDER_OPT is not None and os.path.exists(INPUT_FOLDER_OPT):
        df_opt_array = []
        for folder in os.listdir(INPUT_FOLDER_OPT):
            if os.path.isdir(os.path.join(INPUT_FOLDER_OPT, folder)):
                temp_df = pd.read_csv(
                    INPUT_FOLDER_OPT + "/" + folder + "/" + "data.csv", sep=";"
                )
                df_opt_array.append(adjust_dataframe(temp_df))

        if len(df_opt_array) > 0:
            DF_OPT = pd.concat(df_opt_array)
            DF_OPT = DF_OPT.groupby(DF_OPT.index).mean()
            # DF_OPT = adjust_dataframe(DF_OPT)

        df_opt_std = pd.concat(df_opt_array)
        df_opt_std = df_opt_std.groupby(df_opt_std.index).std()
        # df_opt_std = adjust_dataframe(df_opt_std)
        df_opt_std = df_opt_std.interpolate(method="linear", limit_direction="both")
        df_opt_std = df_opt_std.abs()

        df_std = pd.concat(df_array)
        df_std = df_std.groupby(df_std.index).std()
        # df_std = df_std.interpolate(method='linear', limit_direction='both')
        df_std = df_std.abs()

        fig, ax = plt.subplots()

        ax.errorbar(
            DF["mean_density"],
            DF["mean_flow_spires"],
            yerr=df_std["mean_flow_spires"],
            fmt="o",
            label="Normal",
            capsize=1,
            markersize=2,
            elinewidth=1,
        )
        ax.errorbar(
            DF_OPT["mean_density"],
            DF_OPT["mean_flow_spires"],
            yerr=df_opt_std["mean_flow_spires"],
            fmt="s",
            label="Optimized",
            capsize=1,
            markersize=2,
            elinewidth=1,
        )
        ax.legend(loc="upper right")

        ax.set_xlabel(f"${LABEL_DENSITY} \\ \\left({MU_DENSITY}\\right)$")
        ax.set_ylabel(f"${LABEL_FLOW} \\ \\left({MU_FLOW}\\right)$")
        ax.set_title(f"Fundamental ${LABEL_FLOW}-{LABEL_DENSITY}$ diagram")
        xlim = plt.xlim()
        if PRESENTATION:
            plt.xlim(0, 62)
        else:
            plt.xlim(0, xlim[1] * 1.1)
        ylim = plt.ylim()
        plt.ylim(0, ylim[1] * 1.1)

        x1, x2, y1, y2 = 15, 32, 550, 650  # subregion for tl05
        axins = ax.inset_axes(
            [0.2, 0.01, 0.55, 0.55],
            xlim=(x1, x2),
            ylim=(y1, y2),
            xticklabels=[],
            yticklabels=[],
        )
        axins.errorbar(
            DF["mean_density"],
            DF["mean_flow_spires"],
            yerr=df_std["mean_flow_spires"],
            fmt="o",
            label="Normal",
            capsize=1,
            markersize=2,
            elinewidth=1,
        )
        axins.errorbar(
            DF_OPT["mean_density"],
            DF_OPT["mean_flow_spires"],
            yerr=df_opt_std["mean_flow_spires"],
            fmt="s",
            label="Optimized",
            capsize=1,
            markersize=2,
            elinewidth=1,
        )
        axins.set_xlabel("")
        axins.set_ylabel("")
        axins.legend([], [], frameon=False)  # Remove legend from axins
        axins.patch.set_edgecolor("black")
        axins.patch.set_linewidth(1)

        x_ticks_main = np.array(
            [int(label.get_text()) for label in ax.get_xticklabels()]
        )
        y_ticks_main = np.array(
            [int(label.get_text()) for label in ax.get_yticklabels()]
        )

        # Adjust the tick locations to fit within the limits of the inset axes
        x_ticks_ins = x_ticks_main[(x_ticks_main >= x1) & (x_ticks_main <= x2)]
        y_ticks_ins = y_ticks_main[(y_ticks_main >= y1) & (y_ticks_main <= y2)]

        # Set the tick locations for both axes
        axins.set_xticks(x_ticks_ins)
        axins.set_yticks(y_ticks_ins)

        axins.grid(True)
        ax.indicate_inset_zoom(axins, edgecolor="black")

        if SAVE_FIG:
            plt.savefig(OUT_IMG_FOLDER + "fundamental_qk_opt.pgf")
        plt.show()

    ############################################
    # 3. Fluctuations plot
    ############################################

    max_density_arr = []
    max_value_arr = []
    time_arr = []
    for temp_df in df_array:
        max_density_arr.append(
            temp_df["mean_density"].iloc[temp_df["norm_density_fluctuations"].idxmax()]
        )
        max_value_arr.append(temp_df["norm_density_fluctuations"].max())
        time_arr.append(
            temp_df["time"].iloc[temp_df["norm_density_fluctuations"].idxmax()]
        )
    max_density = (np.mean(max_density_arr), np.std(max_density_arr))
    max_value = (np.mean(max_value_arr), np.std(max_value_arr))
    max_time = (np.mean(time_arr), np.std(time_arr))

    sns.lineplot(x="mean_density", y="norm_density_fluctuations", data=DF)
    # print vertical line at density 42.54357798165134
    plt.axvline(
        K1[0],
        color="black",
        linestyle="--",
        label=f"$\\rho_1 = \\left({K1[0]:.1f} \\pm {K1[1]:.1f}\\right) \\ veh/km$",
    )
    plt.axvspan(K1[0] - K1[1], K1[0] + K1[1], color="lightgrey", alpha=0.35)
    print(f"Max density occurs at time {max_time[0]:.1f} h $\\pm$ {max_time[1]:.1f} h")
    plt.axvline(
        max_density[0],
        color="red",
        linestyle="--",
        label=(
            f"$\\left({{\\sigma_{{\\rho}}/\\rho}}\\right)_{{max}} \\ {{at}} \\ "
            f"\\left({max_density[0]:.1f} \\pm {max_density[1]:.1f} \\right) \\ veh/km$"
        ),
    )
    plt.axvspan(
        max_density[0] - max_density[1],
        max_density[0] + max_density[1],
        color="lightcoral",
        alpha=0.35,
    )

    plt.ylabel(f"$\\sigma_{LABEL_DENSITY}/\\rho$")
    plt.xlabel(f"${LABEL_DENSITY} \\ \\left({MU_DENSITY}\\right)$")

    plt.title("Normalized density fluctuations vs mean density")
    xlim = plt.xlim()
    ylim = plt.ylim()
    if PRESENTATION:
        plt.legend(loc="upper left")
        plt.xlim(0, 62)
        plt.ylim(0, ylim[1] * 1.25)
    else:
        plt.legend(loc="upper right")
        plt.xlim(0, xlim[1] * 1.1)
        plt.ylim(0, ylim[1] * 1.1)
    if SAVE_FIG:
        plt.savefig(OUT_IMG_FOLDER + "density_fluctuations.pgf")
    plt.show()

    ############################################
    # 3.1 Fluctuations plot plot - if exists opt
    ############################################

    if DF_OPT is not None:
        fig, ax = plt.subplots()

        ax.errorbar(
            DF["mean_density"],
            DF["norm_density_fluctuations"],
            yerr=df_std["norm_density_fluctuations"],
            fmt="o",
            label="Normal",
            capsize=1,
            markersize=2,
            elinewidth=1,
        )
        ax.errorbar(
            DF_OPT["mean_density"],
            DF_OPT["norm_density_fluctuations"],
            yerr=df_opt_std["norm_density_fluctuations"],
            fmt="s",
            label="Optimized",
            capsize=1,
            markersize=2,
            elinewidth=1,
        )

        ax.set_ylabel(f"$\\sigma_{LABEL_DENSITY}/\\rho$")
        ax.set_xlabel(f"${LABEL_DENSITY} \\ \\left({MU_DENSITY}\\right)$")

        ax.set_title("Normalized density fluctuations vs mean density")
        ax.legend(loc="upper right")
        xlim = plt.xlim()
        ylim = plt.ylim()
        if PRESENTATION:
            plt.xlim(0, 62)
        else:
            plt.xlim(0, xlim[1] * 1.1)
        plt.ylim(0, ylim[1] * 1.1)

        x1, x2, y1, y2 = 20, 40, 0.8, 1.1  # subregion for tl05
        axins = ax.inset_axes(
            [0.35, 0.01, 0.5, 0.5],
            xlim=(x1, x2),
            ylim=(y1, y2),
            xticklabels=[],
            yticklabels=[],
        )
        axins.errorbar(
            DF["mean_density"],
            DF["norm_density_fluctuations"],
            yerr=df_std["norm_density_fluctuations"],
            fmt="o",
            label="Normal",
            capsize=1,
            markersize=2,
            elinewidth=1,
        )
        axins.errorbar(
            DF_OPT["mean_density"],
            DF_OPT["norm_density_fluctuations"],
            yerr=df_opt_std["norm_density_fluctuations"],
            fmt="s",
            label="Optimized",
            capsize=1,
            markersize=2,
            elinewidth=1,
        )
        axins.set_xlabel("")
        axins.set_ylabel("")
        axins.legend([], [], frameon=False)  # Remove legend from axins
        axins.patch.set_edgecolor("black")
        axins.patch.set_linewidth(1)

        x_ticks_main = np.array(
            [int(float(label.get_text())) for label in ax.get_xticklabels()]
        )
        y_ticks_main = np.array(
            [int(float(label.get_text())) for label in ax.get_yticklabels()]
        )

        # Adjust the tick locations to fit within the limits of the inset axes
        x_ticks_ins = x_ticks_main[(x_ticks_main >= x1) & (x_ticks_main <= x2)]
        y_ticks_ins = y_ticks_main[(y_ticks_main >= y1) & (y_ticks_main <= y2)]

        # Set the tick locations for both axes
        axins.set_xticks(x_ticks_ins)
        axins.set_yticks(y_ticks_ins)

        axins.grid(True)
        ax.indicate_inset_zoom(axins, edgecolor="black")

        if SAVE_FIG:
            plt.savefig(OUT_IMG_FOLDER + "density_fluctuations_opt.pgf")
        plt.show()

    ############################################
    # Signal extraction
    ############################################

    LAG = 500

    THRESHOLD = 5
    INFLUENCE = 0

    first_peak_arr = []
    for temp_df in df_array:
        my_data = temp_df["mean_flow_spires_err"].to_numpy()

        my_data = my_data[15:]

        if "03" in INPUT_FOLDER:
            LAG = len(my_data)
            THRESHOLD = 3

        rp = RealTimePeakDetection(
            my_data[:LAG], lag=LAG, threshold=THRESHOLD, influence=INFLUENCE
        )

        # make time array in hours
        time = temp_df["time"].to_numpy()
        time = time[15:]

        signals = [rp.thresholding_algo(element) for element in my_data]

        signals = np.abs(signals)
        # keep only the first peak
        first_peak_arr.append(np.argmax(signals))

    FIRST = (np.mean(first_peak_arr), np.std(first_peak_arr))
    # print(f"First peak at {FIRST[0]:.1f} h $\\pm$ {FIRST[1]:.1f} h")
    time = DF["time"]
    time = time[15:]

    ############################################
    # 4. Signal plot
    ############################################

    plt.figure(figsize=(8, 5))
    left_ax = sns.lineplot(
        x="time", y="mean_density", data=DF, color="green", label="Mean density $\\rho$"
    )

    plt.title("Comparison of mean density and flow fluctuations over time")
    if max(DF["time"]) < 25:
        plt.xticks(range(0, 25, 1))
    plt.xlabel(f"Time ({MU_TIME})")
    # plot a straight line
    try:
        plt.axvspan(
            time[int(FIRST[0] - FIRST[1])],
            time[int(FIRST[0] + FIRST[1])],
            color="lightcoral",
            alpha=0.35,
        )
        plt.axvline(
            x=time[int(FIRST[0])],
            color="red",
            linestyle="--",
            label=(
                f"$t_p \\ \\left({time[int(FIRST[0])]:.1f} \\pm "
                f"{time[int(FIRST[0] + FIRST[1])] - time[int(FIRST[0])]:.1f} \\right) \\ h$"
            ),
        )
    except IndexError as e:
        print(f"An IndexError occurred: {e}")
    except ValueError as e:
        print(f"A ValueError occurred: {e}")
    except TypeError as e:
        print(f"A TypeError occurred: {e}")
    plt.axvline(
        x=TK1[0],
        color="black",
        linestyle="--",
        label=f"$t_1 = \\left({TK1[0]:.1f} \\pm {TK1[1]:.1f} \\right) \\ h$",
    )
    plt.axvspan(TK1[0] - TK1[1], TK1[0] + TK1[1], color="lightgrey", alpha=0.35)
    left_ax.legend(loc="upper left")

    right_ax = left_ax.twinx()
    sns.lineplot(
        x="time",
        y="norm_flow_spires_fluctuations",
        data=DF,
        color="blue",
        ax=right_ax,
        label="Flow fluctuations $\\sigma_\\phi$",
    )

    right_ax.legend(loc="upper right")
    right_ax.grid(False)

    right_ax_lim = right_ax.get_ylim()
    left_ax_lim = left_ax.get_ylim()
    if PRESENTATION:
        right_ax.set(ylim=(right_ax_lim[0], 350))
        left_ax.set(ylim=(left_ax_lim[0], 90))
    else:
        left_ax.set(ylim=(left_ax_lim[0], left_ax_lim[1] * 1.1))
        right_ax.set(ylim=(right_ax_lim[0], right_ax_lim[1] * 1.1))

    left_ax.set_ylabel(f"${LABEL_DENSITY} \\ \\left({MU_DENSITY}\\right)$")
    right_ax.set_ylabel(f"$\\sigma_{LABEL_FLOW} \\ \\left({MU_FLOW}\\right)$")

    xlim = plt.xlim()
    ylim = plt.ylim()
    if PRESENTATION:
        plt.xlim(0, 45)  # 45, 100, 48.5
    else:
        plt.xlim(0, xlim[1] * 1.1)

    if SAVE_FIG:
        plt.savefig(OUT_IMG_FOLDER + "mean_density_vs_flow_fluctuations_peak.pgf")
    plt.show()
