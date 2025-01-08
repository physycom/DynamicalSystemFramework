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


def merge_edges(
    graph: nx.DiGraph, previous_node: int, successive_node: int, node: int
) -> dict:
    """
    Merge two edges into a single edge.
    The function merges the edges into a single edge if the following conditions are met:
    - the name of the two edges is the same
    - the number of lanes is the same
    - the geometry of the two edges is contiguous
    - the coordinates of the previous_node and successive_node are in the geometry

    Parameters:
    ----------------
    graph (networkx.DiGraph): the graph
    previous_node (int): the previous node
    successive_node (int): the successive node
    node (int): the id of the node which will be removed

    Returns:
    ----------------
    dict: the new edge
    """
    try:
        data_u = graph.get_edge_data(previous_node, node)[0]
        data_v = graph.get_edge_data(node, successive_node)[0]
        data_u.setdefault("lanes", 1)
        data_v.setdefault("lanes", 1)
        if (
            not (data_u["name"] in data_v["name"] or data_v["name"] in data_u["name"])
            or data_u["lanes"] != data_v["lanes"]
        ):
            return None
        edge_uv = data_u.copy()
        # set length as the sum
        edge_uv["length"] = data_u["length"] + data_v["length"]
        edge_uv["lanes"] = int(data_u["lanes"])
        # merge also linestrings
        edge_uv["geometry"] = data_u["geometry"].union(data_v["geometry"])
        if isinstance(edge_uv["geometry"], MultiLineString):
            edge_uv["geometry"] = linemerge(edge_uv["geometry"])
        else:
            edge_uv["geometry"] = edge_uv["geometry"]
        if isinstance(edge_uv["geometry"], LineString):
            coords = list(edge_uv["geometry"].coords)
        else:
            # If it's still a MultiLineString,
            # handle it by iterating through its individual LineStrings
            coords = []
            for line in edge_uv["geometry"]:
                coords.extend(
                    list(line.coords)
                )  # Add coords from each individual LineString
        # take the list from coordinates of previous_node to coordinates of successive_node
        u_coords = (graph.nodes[previous_node]["x"], graph.nodes[previous_node]["y"])
        v_coords = (
            graph.nodes[successive_node]["x"],
            graph.nodes[successive_node]["y"],
        )
        if u_coords not in coords or v_coords not in coords:
            return None
        # cut coords from u_index to v_index
        edge_uv["geometry"] = LineString(
            coords[coords.index(u_coords) : coords.index(v_coords)]
        )
    except TypeError:
        # type error means that data_u or data_v cannot be created,
        # which means that the road is a one-way road
        # thus, skip the type error
        return None

    return edge_uv


def simplify_graph(graph_original: nx.DiGraph) -> nx.DiGraph:
    """
    Simplify the graph by removing nodes that have only two neighborsand are actually the same
    street.
    The function merges the edges into a single edge.

    Parameters:
    ----------------
    graph_original (networkx.DiGraph): the graph to simplify

    Returns:
    ----------------
    networkx.DiGraph: the simplified graph
    """
    graph = graph_original.copy()
    previous_nodes = 0
    while previous_nodes != len(graph.nodes):
        logging.info("New cycle: current_nodes=%d", len(graph.nodes))
        previous_nodes = len(graph.nodes)
        for node in graph.copy().nodes:
            # define neighborus as list of predecessors and successors
            neighbours = list(graph.predecessors(node)) + list(graph.successors(node))
            if (
                len(neighbours) != 2
                or graph.in_degree(node) != graph.out_degree(node)
                or graph.in_degree(node) > 2
            ):
                continue
            u, v = neighbours
            if graph.has_edge(u, v):
                continue
            edge_uv = merge_edges(graph, u, v, node)
            edge_vu = merge_edges(graph, v, u, node)

            if not (edge_uv is None and edge_vu is None):
                if edge_uv is not None:
                    # print(f"Edges {u} -> {node} and {node} -> {v} can be merged.")
                    graph.add_edge(
                        u,
                        v,
                        length=edge_uv["length"],
                        name=edge_uv["name"],
                        geometry=edge_uv["geometry"],
                    )
                    # print(f"Added edge {graph.get_edge_data(u, v)}")
                if edge_vu is not None:
                    # print(f"Edges {v} -> {node} and {node} -> {u} can be merged.")
                    graph.add_edge(
                        v,
                        u,
                        length=edge_vu["length"],
                        name=edge_vu["name"],
                        geometry=edge_vu["geometry"],
                    )
                    # print(f"Added edge {graph.get_edge_data(v, u)}")
                graph.remove_node(node)
                # print(f"Removed node {node}")

    # Remove all nodes that are not in the giant component
    graph.remove_nodes_from(
        set(graph.nodes) - max(list(nx.weakly_connected_components(graph)), key=len)
    )
    # remove all self-loops
    graph.remove_edges_from(list(nx.selfloop_edges(graph)))
    # check if there are edges with same u and v. If true, keep only the one with the bigger lanes
    edges_to_remove = []
    seen_edges = {}

    for u, v, data in graph.edges(data=True):
        lanes = data.get("lanes", 0)

        if (u, v) not in seen_edges:
            seen_edges[(u, v)] = (lanes, None)  # Store first edge and its lanes count
        else:
            existing_lanes, existing_edge = seen_edges[(u, v)]

            if lanes > existing_lanes:
                edges_to_remove.append(
                    existing_edge
                )  # Remove the previous edge if the current one has more lanes
                seen_edges[(u, v)] = (lanes, (u, v))  # Update to keep current edge
            else:
                edges_to_remove.append(
                    (u, v)
                )  # Remove the current edge if it has fewer 'lanes'

    graph.remove_edges_from(edges_to_remove)

    return graph


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

    # get the street network for San Cesario sul Panaro
    G_ALL = ox.graph_from_place(parser.place, network_type="drive")
    logging.info(
        "Graph created with %d nodes and %d edges.", len(G_ALL.nodes), len(G_ALL.edges)
    )

    gdf_nodes, gdf_edges = ox.graph_to_gdfs(G_ALL)
    gdf_edges["highway"] = gdf_edges["highway"].apply(
        lambda x: x[-1] if isinstance(x, list) else x
    )
    if "lanes" not in gdf_edges.columns:
        gdf_edges["lanes"] = 1
    gdf_edges["lanes"] = gdf_edges["lanes"].apply(
        lambda x: max(x) if isinstance(x, list) else 1 if pd.isna(x) else x
    )
    gdf_edges["name"] = gdf_edges["name"].apply(
        lambda x: " ".join(x) if isinstance(x, list) else " " if pd.isna(x) else x
    )
    # gdf_edges = gdf_edges[~gdf_edges["access"].isin(["no", "private"])]

    # Make a plot to visualize the removed links
    removed_patch = mpatches.Patch(color=RGBA_RED, label="Removed Nodes and Edges")

    if parser.exclude_motorway:
        gdf_edges = gdf_edges[~gdf_edges["highway"].isin(FLAGS_MOTORWAY)]
    if parser.exclude_residential:
        gdf_edges = gdf_edges[~gdf_edges["highway"].isin(FLAGS_RESIDENTIAL)]

    # rebuild the graph
    G = ox.graph_from_gdfs(gdf_nodes, gdf_edges)
    G.remove_nodes_from(list(nx.isolates(G)))
    logging.info(
        "Graph filtered: now it has %d nodes and %d edges.", len(G.nodes), len(G.edges)
    )
    G = simplify_graph(G)
    logging.info(
        "Graph simplified: now it has %d nodes and %d edges.",
        len(G.nodes),
        len(G.edges),
    )
    # assert that graph has not isolated nodes
    assert not list(nx.isolates(G))
    # assert that graph has not self-loops
    assert not list(nx.selfloop_edges(G))

    fig, ax = ox.plot_graph(
        G_ALL,
        node_color=[
            RGBA_RED if node not in G.nodes else RGBA_WHITE for node in G_ALL.nodes
        ],
        edge_color=[
            RGBA_RED if edge not in G.edges else RGBA_WHITE for edge in G_ALL.edges
        ],
        show=False,
        close=False,
    )
    ax.legend(handles=[removed_patch])
    fig.set_size_inches(16, 9)
    plt.savefig("removed_nodes_and_edges.png")

    # Plot resulting graph
    fig, ax = ox.plot_graph(G, show=False, close=False)
    fig.set_size_inches(16, 9)
    plt.savefig("final_graph.png")

    gdf_nodes, gdf_edges = ox.graph_to_gdfs(G)
    # notice that osmnid is the index of the gdf_nodes DataFrame, so take it as a column
    gdf_nodes.reset_index(inplace=True)
    gdf_edges.reset_index(inplace=True)

    # assert that there are no edges with the same u and v
    assert not gdf_edges.duplicated(subset=["u", "v"]).any()
    # Prepare node dataframe
    gdf_nodes = gdf_nodes[["osmid", "x", "y", "highway"]]
    # Prepare edge dataframe

    gdf_edges.to_csv("edges_ALL.csv", sep=";", index=False)
    gdf_edges = gdf_edges[
        ["u", "v", "length", "oneway", "lanes", "highway", "maxspeed", "name"]
    ]
    # Save the data
    gdf_nodes.to_csv("nodes.csv", sep=";", index=False)
    gdf_edges.to_csv("edges.csv", sep=";", index=False)
