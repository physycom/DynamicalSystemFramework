import networkx as nx
import osmnx as ox


def get_cartography(
    place_name: str,
    network_type: str = "drive",
    consolidate_intersections: bool | float = 10,
    dead_ends: bool = False,
    return_type: str = "gdfs",
) -> tuple | nx.MultiDiGraph:
    """
    Retrieves and processes cartography data for a specified place using OpenStreetMap data.

    This function downloads a street network graph for the given place, optionally consolidates
    intersections to simplify the graph, removes edges with zero length, self-loops and isolated nodes,
    and standardizes the attribute names in the graph. Can return either GeoDataFrames or the graph itself.

    Args:
        place_name (str): The name of the place (e.g., city, neighborhood) to retrieve cartography for.
        network_type (str, optional): The type of network to retrieve. Common values include "drive",
            "walk", "bike". Defaults to "drive".
        consolidate_intersections (bool | float, optional): If True, consolidates intersections using
            a default tolerance. If a float, uses that value as the tolerance for consolidation.
            Set to False to skip consolidation. Defaults to 10.
        dead_ends (bool, optional): Whether to include dead ends when consolidating intersections.
            Only relevant if consolidate_intersections is enabled. Defaults to False.
        return_type (str, optional): Type of return value. Options are "gdfs" (GeoDataFrames) or
            "graph" (NetworkX MultiDiGraph). Defaults to "gdfs".

    Returns:
        tuple | nx.MultiDiGraph: If return_type is "gdfs", returns a tuple containing two GeoDataFrames:
            - gdf_edges: GeoDataFrame with processed edge data, including columns like 'source',
              'target', 'nlanes', 'type', 'name', 'id', and 'geometry'.
            - gdf_nodes: GeoDataFrame with processed node data, including columns like 'id', 'type',
              and 'geometry'.
            If return_type is "graph", returns the NetworkX MultiDiGraph with standardized attributes.
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
    G.remove_nodes_from(list(nx.isolates(G)))

    # Standardize edge attributes in the graph
    edges_to_update = []
    for u, v, k, data in G.edges(keys=True, data=True):
        edge_updates = {}

        # Standardize lanes
        if "lanes" in data:
            lanes_value = data["lanes"]
            if isinstance(lanes_value, list):
                edge_updates["nlanes"] = min(lanes_value)
            else:
                edge_updates["nlanes"] = lanes_value
            edge_updates["_remove_lanes"] = True
        else:
            edge_updates["nlanes"] = 1

        # Standardize highway -> type
        if "highway" in data:
            edge_updates["type"] = data["highway"]
            edge_updates["_remove_highway"] = True

        # Standardize name
        if "name" in data:
            name_value = data["name"]
            if isinstance(name_value, list):
                name_value = ",".join(name_value)
            edge_updates["name"] = str(name_value).lower().replace(" ", "_")
        else:
            edge_updates["name"] = "unknown"

        # Remove unnecessary attributes
        for attr in [
            "bridge",
            "tunnel",
            "access",
            "service",
            "ref",
            "reversed",
            "junction",
            "osmid",
        ]:
            if attr in data:
                edge_updates[f"_remove_{attr}"] = True

        if consolidate_intersections:
            for attr in ["u_original", "v_original"]:
                if attr in data:
                    edge_updates[f"_remove_{attr}"] = True

        edges_to_update.append((u, v, k, edge_updates))

    # Apply edge updates
    for u, v, k, updates in edges_to_update:
        for key, value in updates.items():
            if key.startswith("_remove_"):
                attr_name = key.replace("_remove_", "")
                if attr_name in G[u][v][k]:
                    del G[u][v][k][attr_name]
            else:
                G[u][v][k][key] = value

    # Standardize node attributes in the graph
    nodes_to_update = []
    for node, data in G.nodes(data=True):
        node_updates = {}

        # Standardize osmid -> id (keep both for compatibility with ox.graph_to_gdfs)
        if "osmid" in data:
            node_updates["id"] = data["osmid"]

        # Standardize highway -> type
        if "highway" in data:
            node_updates["type"] = data["highway"]
            node_updates["_remove_highway"] = True
        else:
            # Set type to "N/A" if not present
            node_updates["type"] = "N/A"

        # Remove unnecessary attributes
        for attr in ["street_count", "ref", "cluster", "junction"]:
            if attr in data:
                node_updates[f"_remove_{attr}"] = True

        if consolidate_intersections and "osmid_original" in data:
            node_updates["_remove_osmid_original"] = True

        nodes_to_update.append((node, node_updates))

    # Apply node updates
    for node, updates in nodes_to_update:
        for key, value in updates.items():
            if key.startswith("_remove_"):
                attr_name = key.replace("_remove_", "")
                if attr_name in G.nodes[node]:
                    del G.nodes[node][attr_name]
            else:
                G.nodes[node][key] = value

    # Fill NaN values in node type attribute
    for node in G.nodes():
        if (
            "type" not in G.nodes[node]
            or G.nodes[node]["type"] is None
            or (
                isinstance(G.nodes[node]["type"], float)
                and G.nodes[node]["type"] != G.nodes[node]["type"]
            )
        ):  # Check for NaN
            G.nodes[node]["type"] = "N/A"

    # Convert to lat/long
    G = ox.project_graph(G, to_latlong=True)

    # Return graph or GeoDataFrames based on return_type
    if return_type == "graph":
        return G
    else:  # return_type == "gdfs"
        # Convert back to MultiDiGraph temporarily for ox.graph_to_gdfs compatibility
        gdf_nodes, gdf_edges = ox.graph_to_gdfs(G)

        # Reset index and add edge id
        gdf_edges.reset_index(inplace=True)
        gdf_edges["id"] = gdf_edges.index
        gdf_edges.rename(columns={"u": "source", "v": "target"}, inplace=True)
        gdf_edges.drop(columns=["key"], inplace=True, errors="ignore")

        # Reset index for nodes
        gdf_nodes.reset_index(inplace=True)
        gdf_nodes.drop(columns=["y", "x"], inplace=True, errors="ignore")
        gdf_nodes.rename(columns={"osmid": "id"}, inplace=True)

        return gdf_edges, gdf_nodes


# if __name__ == "__main__":
#     Produce data for tests
#     edges, nodes = get_cartography("Postua, Piedmont, Italy", consolidate_intersections=False)
#     edges.to_csv("../../../test/data/postua_edges.csv", index=False, sep=";")
#     edges.to_file("../../../test/data/postua_edges.geojson", index=False, driver="GeoJSON")
#     nodes.to_csv("../../../test/data/postua_nodes.csv", index=False, sep=";")
#     edges, nodes = get_cartography("Forlì, Emilia-Romagna, Italy")
#     edges.to_csv("../../../test/data/forlì_edges.csv", index=False, sep=";")
#     nodes.to_csv("../../../test/data/forlì_nodes.csv", index=False, sep=";")
