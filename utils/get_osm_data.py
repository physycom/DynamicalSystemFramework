"""
This script is used to get the OSM data of a place and save it in a csv file.
The place is passed as a command line argument.

Example:
python get_osm_data.py -p "Bologna, Emilia-Romagna, Italy" -er

The output files are:
- {place}_nodes.csv
- {place}_edges.csv

The files are saved in the current directory.
"""

from argparse import ArgumentParser
from pathlib import Path
import ast
import logging
import osmnx as ox

__version__ = "2025.2.11"

RGBA_RED = (1, 0, 0, 1)
RGBA_WHITE = (1, 1, 1, 1)

FLAGS_MOTORWAY = ["motorway", "motorway_link"]
FLAGS_NORMAL = [
    "primary",
    "secondary",
    "tertiary",
    "trunk",
    "primary_link",
    "secondary_link",
    "tertiary_link",
    "trunk_link",
]
FLAGS_RESIDENTIAL = [
    "residential",
    "living_street",
    "unclassified",
    "service",
    "pedestrian",
    "busway",
]

if __name__ == "__main__":
    parser = ArgumentParser("Script to get the OSM data of a place.")
    parser.add_argument(
        "-p",
        "--place",
        required=True,
        help="Place to get the OSM data in the format: city, province, country",
    )
    parser.add_argument(
        "-em",
        "--exclude-motorway",
        action="store_true",
        help="Exclude motorways from the data. Default is False",
    )
    parser.add_argument(
        "-er",
        "--exclude-residential",
        action="store_true",
        help="Exclude residential roads from the data. Default is False",
    )
    parser.add_argument(
        "--allow-duplicates",
        action="store_true",
        help="Allow duplicated edges in the data. Default is False",
    )
    parser.add_argument(
        "-t",
        "--tolerance",
        type=int,
        default=20,
        help="Radius in meters to merge intersections. For more info, see osmnx documentation.",
    )
    parser.add_argument(
        "--use-original-ids", action="store_true", help="Use the original ids from OSM."
    )
    parser.add_argument(
        "-of",
        "--output-folder",
        type=str,
        default=".",
        help="Folder where the output files will be saved. Default is the current folder.",
    )
    parser.add_argument(
        "-c",
        "--consolidate-intersections",
        action="store_true",
        help="Consolidate intersections. Default is False",
    )
    parser.add_argument(
        "--keep-geometry",
        action="store_true",
        help="Keep the GEOMETRY column in aoutput csv.",
    )
    parser = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
    # set up colored logging
    logging.addLevelName(
        logging.INFO, f"\033[1;32m{logging.getLevelName(logging.INFO)}\033[1;0m"
    )
    logging.addLevelName(
        logging.WARNING, f"\033[1;33m{logging.getLevelName(logging.WARNING)}\033[1;0m"
    )
    logging.addLevelName(
        logging.ERROR, f"\033[1;31m{logging.getLevelName(logging.ERROR)}\033[1;0m"
    )

    logging.info("Welcome to get_osm_data.py v%s", __version__)

    # define CUSTOM_FILTER basing on FLAGS and args
    FLAGS = FLAGS_NORMAL
    if not parser.exclude_motorway:
        FLAGS += FLAGS_MOTORWAY
    if not parser.exclude_residential:
        FLAGS += FLAGS_RESIDENTIAL
    if not parser.exclude_motorway and not parser.exclude_residential:
        FLAGS = None
    if FLAGS is None:
        CUSTOM_FILTER = None
    else:
        CUSTOM_FILTER = f"[\"highway\"~\"{'|'.join(FLAGS)}\"]"
    logging.info("Custom filter: %s", CUSTOM_FILTER)
    GRAPH = ox.graph_from_place(parser.place, network_type="drive")
    ox.plot_graph(
        GRAPH,
        show=False,
        close=True,
        save=True,
        filepath=Path(parser.output_folder) / "original.png",
    )
    logging.info(
        "Original network has %d nodes and %d edges.",
        len(GRAPH.nodes),
        len(GRAPH.edges),
    )
    if FLAGS is not None:
        GRAPH = ox.graph_from_place(
            parser.place, network_type="drive", custom_filter=CUSTOM_FILTER
        )
        logging.info(
            "Custom filtered graph has %d nodes and %d edges.",
            len(GRAPH.nodes),
            len(GRAPH.edges),
        )
    if parser.consolidate_intersections:
        GRAPH = ox.consolidate_intersections(
            ox.project_graph(GRAPH), tolerance=parser.tolerance
        )
        logging.info(
            "Consolidated graph has %d nodes and %d edges.",
            len(GRAPH.nodes),
            len(GRAPH.edges),
        )
    # plot graph on a 16x9 figure and save into file
    ox.plot_graph(
        GRAPH,
        show=False,
        close=True,
        save=True,
        filepath=Path(parser.output_folder) / "final.png",
    )
    gdf_nodes, gdf_edges = ox.graph_to_gdfs(ox.project_graph(GRAPH, to_latlong=True))
    # notice that osmnid is the index of the gdf_nodes DataFrame, so take it as a column
    gdf_nodes.reset_index(inplace=True)
    gdf_edges.reset_index(inplace=True)

    if parser.use_original_ids:
        for index, row in gdf_nodes.iterrows():
            # if "osmid_original" is a list, keep the first element
            old_list = ast.literal_eval(str(row["osmid_original"]))
            if isinstance(old_list, list):
                new_id = old_list[0]
                # update the edges with u_original or v_original in old_list, with new_id
                gdf_edges.loc[gdf_edges["u_original"].isin(old_list), "u_original"] = (
                    new_id
                )
                gdf_edges.loc[gdf_edges["v_original"].isin(old_list), "v_original"] = (
                    new_id
                )
                # update the node with new_id
                gdf_nodes.loc[index, "osmid_original"] = new_id
        if not parser.keep_geometry:
            gdf_nodes = gdf_nodes[["osmid_original", "x", "y", "highway"]]
            gdf_edges = gdf_edges[
                [
                    "u_original",
                    "v_original",
                    "length",
                    "lanes",
                    "highway",
                    "maxspeed",
                    "name",
                ]
            ]
        else:
            gdf_nodes = gdf_nodes[["osmid_original", "x", "y", "highway", "geometry"]]
            gdf_edges = gdf_edges[
                [
                    "u_original",
                    "v_original",
                    "length",
                    "lanes",
                    "highway",
                    "maxspeed",
                    "name",
                    "geometry",
                ]
            ]

    else:
        if not "lanes" in gdf_edges.columns:
            gdf_edges["lanes"] = 1
        if not parser.keep_geometry:
            gdf_nodes = gdf_nodes[["osmid", "x", "y", "highway"]]
            gdf_edges = gdf_edges[
                ["u", "v", "length", "lanes", "highway", "maxspeed", "name"]
            ]
        else:
            gdf_nodes = gdf_nodes[["osmid", "x", "y", "highway", "geometry"]]
            gdf_edges = gdf_edges[
                ["u", "v", "length", "lanes", "highway", "maxspeed", "name", "geometry"]
            ]

    if parser.allow_duplicates:
        N_DUPLICATES = 0
    else:
        # Check for duplicate edges
        if parser.use_original_ids:
            duplicated_mask = gdf_edges.duplicated(subset=["u_original", "v_original"])
        else:
            duplicated_mask = gdf_edges.duplicated(subset=["u", "v"])
        N_DUPLICATES = duplicated_mask.sum()

    if N_DUPLICATES > 0:
        logging.warning(
            "There are %d duplicated edges which will be removed. "
            "Please look at them in the promped plot.",
            N_DUPLICATES,
        )
        # Plot the graph with duplicated edges in red
        edge_colors = [
            RGBA_RED if duplicated_mask.iloc[i] else RGBA_WHITE
            for i in range(len(gdf_edges))
        ]
        ox.plot_graph(GRAPH, edge_color=edge_colors)

        # Remove duplicated edges
        if parser.use_original_ids:
            gdf_edges = gdf_edges.drop_duplicates(subset=["u_original", "v_original"])
        else:
            gdf_edges = gdf_edges.drop_duplicates(subset=["u", "v"])

    # drop self loops
    if parser.use_original_ids:
        gdf_edges = gdf_edges[gdf_edges["u_original"] != gdf_edges["v_original"]]
    else:
        gdf_edges = gdf_edges[gdf_edges["u"] != gdf_edges["v"]]

    # Save the data
    place = parser.place.split(",")[0].strip().lower()
    gdf_nodes.to_csv(
        Path(parser.output_folder) / f"{place}_nodes.csv", sep=";", index=False
    )
    logging.info(
        'Nodes correctly saved in "%s"',
        Path(parser.output_folder) / f"{place}_nodes.csv",
    )
    gdf_edges.to_csv(
        Path(parser.output_folder) / f"{place}_edges.csv", sep=";", index=False
    )
    logging.info(
        'Edges correctly saved in "%s"',
        Path(parser.output_folder) / f"{place}_edges.csv",
    )
