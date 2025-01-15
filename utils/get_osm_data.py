"""
This script is used to get the OSM data of a place and save it in a csv file.
The place is passed as a command line argument.

Example:
python get_osm_data.py --place "Bologna, Emilia-Romagna, Italy" --exclude-residential

The output files are:
- nodes.csv
- edges.csv

The files are saved in the current directory.
"""

from argparse import ArgumentParser
import logging
import osmnx as ox
import networkx as nx
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import pandas as pd
from shapely.geometry import MultiLineString, LineString
from shapely.ops import linemerge

RGBA_RED = (1, 0, 0, 0.3)
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
        "--place",
        required=True,
        help="Place to get the OSM data in the format: city, province, country",
    )
    parser.add_argument(
        "--exclude-motorway",
        action="store_true",
        help="Exclude motorways from the data",
    )
    parser.add_argument(
        "--exclude-residential",
        action="store_true",
        help="Exclude residential roads from the data",
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

    # define CUSTOM_FILTER basing on FLAGS and args
    FLAGS = FLAGS_NORMAL
    if not parser.exclude_motorway:
        FLAGS += FLAGS_MOTORWAY
    if not parser.exclude_residential:
        FLAGS += FLAGS_RESIDENTIAL
    CUSTOM_FILTER = f"[\"highway\"~\"{'|'.join(FLAGS)}\"]"
    logging.info("Custom filter: %s", CUSTOM_FILTER)
    G_ALL = ox.graph_from_place(
        parser.place, network_type="drive", custom_filter=CUSTOM_FILTER
    )
    logging.info(
        "Graph created with %d nodes and %d edges.", len(G_ALL.nodes), len(G_ALL.edges)
    )
    G_ALL = ox.consolidate_intersections(ox.project_graph(G_ALL), tolerance=20)
    logging.info(
        "Graph consolidated with %d nodes and %d edges.",
        len(G_ALL.nodes),
        len(G_ALL.edges),
    )
    ox.plot_graph(G_ALL)
    gdf_nodes, gdf_edges = ox.graph_to_gdfs(ox.project_graph(G_ALL, to_latlong=True))
    # notice that osmnid is the index of the gdf_nodes DataFrame, so take it as a column
    gdf_nodes.reset_index(inplace=True)
    gdf_edges.reset_index(inplace=True)

    # Prepare node dataframe
    gdf_nodes = gdf_nodes[["osmid", "x", "y", "highway"]]
    # Prepare edge dataframe

    gdf_edges.to_csv("edges_ALL.csv", sep=";", index=False)
    gdf_edges = gdf_edges[
        ["u", "v", "length", "oneway", "lanes", "highway", "maxspeed", "name"]
    ]
    # warn for duplicate edges
    if gdf_edges.duplicated(subset=["u", "v"]).sum() > 0:
        logging.warning(
            "There are %d duplicated edges. They will be removed.",
            gdf_edges.duplicated(subset=["u", "v"]).sum(),
        )
        gdf_edges = gdf_edges.drop_duplicates(subset=["u", "v"])
    # Save the data
    gdf_nodes.to_csv("nodes.csv", sep=";", index=False)
    gdf_edges.to_csv("edges.csv", sep=";", index=False)
