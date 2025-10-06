import osmnx as ox


def get_cartography(
    place_name: str,
    network_type: str = "drive",
    consolidate_intersections: bool | float = 10,
    dead_ends: bool = False,
) -> tuple:
    """
    Retrieves and processes cartography data for a specified place using OpenStreetMap data.

    This function downloads a street network graph for the given place, optionally consolidates
    intersections to simplify the graph, removes edges with zero length, self-loops and isolated nodes, and converts the graph
    into GeoDataFrames for edges and nodes with standardized column names and data types.

    Args:
        place_name (str): The name of the place (e.g., city, neighborhood) to retrieve cartography for.
        network_type (str, optional): The type of network to retrieve. Common values include "drive",
            "walk", "bike". Defaults to "drive".
        consolidate_intersections (bool | float, optional): If True, consolidates intersections using
            a default tolerance. If a float, uses that value as the tolerance for consolidation.
            Set to False to skip consolidation. Defaults to 10.
        dead_ends (bool, optional): Whether to include dead ends when consolidating intersections.
            Only relevant if consolidate_intersections is enabled. Defaults to False.

    Returns:
        tuple: A tuple containing two GeoDataFrames:
            - gdf_edges: GeoDataFrame with processed edge data, including columns like 'source',
              'target', 'nlanes', 'type', 'name', 'id', and 'geometry'.
            - gdf_nodes: GeoDataFrame with processed node data, including columns like 'id', 'type',
              and 'geometry'.
    """
    if consolidate_intersections and isinstance(consolidate_intersections, bool):
        consolidate_intersections = 10  # Default tolerance value
    G = ox.graph_from_place(place_name, network_type=network_type)
    if consolidate_intersections:
        G = ox.consolidate_intersections(
            ox.project_graph(G),
            tolerance=consolidate_intersections,
            rebuild_graph=True,
            dead_ends=dead_ends,
        )
    # Remove all edges with length 0
    G.remove_edges_from(
        [
            (u, v, k)
            for u, v, k, data in G.edges(keys=True, data=True)
            if data.get("length", 0) == 0
        ]
    )
    # Remove self-loops
    G.remove_edges_from([(u, v, k) for u, v, k in G.edges(keys=True) if u == v])
    # Remove also isolated nodes
    G.remove_nodes_from(list(ox.isolate_nodes(G)))

    # Create GeoDataFrames
    gdf_nodes, gdf_edges = ox.graph_to_gdfs(ox.project_graph(G, to_latlong=True))

    #####################################################
    # Preparing gdf_edges
    gdf_edges.reset_index(inplace=True)

    if consolidate_intersections:
        gdf_edges["u"] = gdf_edges["u_original"]
        gdf_edges["v"] = gdf_edges["v_original"]
        gdf_edges.drop(columns=["u_original", "v_original"], inplace=True)
    gdf_edges["id"] = gdf_edges.index
    gdf_edges.drop(
        columns=[
            "key",
            "bridge",
            "tunnel",
            "access",
            "service",
            "ref",
            "reversed",
            "junction",
            "osmid",
        ],
        inplace=True,
    )
    gdf_edges.rename(
        columns={"u": "source", "v": "target", "lanes": "nlanes", "highway": "type"},
        inplace=True,
    )

    gdf_edges["nlanes"] = gdf_edges["nlanes"].fillna(1)
    gdf_edges["nlanes"] = gdf_edges["nlanes"].apply(
        lambda x: min(x) if isinstance(x, list) else x
    )

    gdf_edges["name"] = gdf_edges["name"].fillna("unknown")
    gdf_edges["name"] = gdf_edges["name"].apply(
        lambda x: ",".join(x) if isinstance(x, list) else x
    )
    gdf_edges["name"] = gdf_edges["name"].str.lower().str.replace(" ", "_")

    #####################################################
    # Preparing gdf_nodes
    gdf_nodes.reset_index(inplace=True)
    if consolidate_intersections:
        gdf_nodes["osmid"] = gdf_nodes["osmid_original"]
        gdf_nodes.drop(columns=["osmid_original"], inplace=True)
    gdf_nodes.drop(
        columns=["y", "x", "street_count", "ref", "cluster", "junction"], inplace=True
    )
    gdf_nodes.rename(columns={"osmid": "id", "highway": "type"}, inplace=True)

    return gdf_edges, gdf_nodes
