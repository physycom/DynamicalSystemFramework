"""
Tests for cartography module.
"""

import networkx as nx
from dsf.python.cartography import get_cartography, graph_to_gdfs, graph_from_gdfs


def test_consistency():
    """
    A simple consistency test to verify that converting from GeoDataFrames to graph and back
    yields the same GeoDataFrames.
    """
    G_CART = get_cartography("Postua, Piedmont, Italy", return_type="graph")
    edges_cart, nodes_cart = get_cartography(
        "Postua, Piedmont, Italy", return_type="gdfs"
    )

    edges, nodes = graph_to_gdfs(G_CART)

    assert edges_cart.equals(edges), (
        "Edges GeoDataFrames are not equal after conversion."
    )
    assert nodes_cart.equals(nodes), (
        "Nodes GeoDataFrames are not equal after conversion."
    )

    G = graph_from_gdfs(edges_cart, nodes_cart)

    # Check structure first
    assert nx.is_isomorphic(G_CART, G), "Graphs are not isomorphic after conversion."
