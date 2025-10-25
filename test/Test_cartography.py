"""
Tests for cartography module.
"""

import pytest
import networkx as nx
from dsf.python.cartography import (
    get_cartography,
    graph_to_gdfs,
    graph_from_gdfs,
    create_manhattan_cartography,
)


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


class TestCreateManhattanCartography:
    """Tests for create_manhattan_cartography function."""

    def test_default_parameters(self):
        """Test creation with default parameters."""
        edges, nodes = create_manhattan_cartography()

        # Check that we get the expected number of nodes (10x10 = 100)
        assert len(nodes) == 100
        # Check that we get bidirectional edges: (10-1)*10 + 10*(10-1) = 90 + 90 = 180 edges, times 2 for bidirectional = 360
        assert len(edges) == 360

        # Check that nodes have required columns
        assert "id" in nodes.columns
        assert "type" in nodes.columns
        assert "geometry" in nodes.columns

        # Check that edges have required columns
        assert "id" in edges.columns
        assert "source" in edges.columns
        assert "target" in edges.columns
        assert "nlanes" in edges.columns
        assert "type" in edges.columns
        assert "name" in edges.columns
        assert "length" in edges.columns
        assert "geometry" in edges.columns
        assert "maxspeed" in edges.columns

    def test_custom_grid_size(self):
        """Test creation with custom grid size."""
        n_x, n_y = 5, 4
        edges, nodes = create_manhattan_cartography(n_x=n_x, n_y=n_y)

        # Check node count
        assert len(nodes) == n_x * n_y

        # Calculate expected edge count
        # Horizontal edges: n_y * (n_x - 1) * 2 (bidirectional)
        # Vertical edges: n_x * (n_y - 1) * 2 (bidirectional)
        expected_edges = 2 * (n_y * (n_x - 1) + n_x * (n_y - 1))
        assert len(edges) == expected_edges

    def test_node_ids_are_unique(self):
        """Test that all node IDs are unique."""
        edges, nodes = create_manhattan_cartography(n_x=5, n_y=5)

        node_ids = nodes["id"].tolist()
        assert len(node_ids) == len(set(node_ids))

    def test_edge_ids_are_unique(self):
        """Test that all edge IDs are unique."""
        edges, nodes = create_manhattan_cartography(n_x=5, n_y=5)

        edge_ids = edges["id"].tolist()
        assert len(edge_ids) == len(set(edge_ids))

    def test_all_edges_are_bidirectional(self):
        """Test that for every edge u->v there exists an edge v->u."""
        edges, nodes = create_manhattan_cartography(n_x=4, n_y=4)

        # Create a set of (source, target) pairs
        edge_pairs = set(zip(edges["source"], edges["target"]))

        # Check that for every (u, v) there exists (v, u)
        for source, target in edge_pairs:
            assert (target, source) in edge_pairs

    def test_node_types(self):
        """Test that all nodes have the correct type."""
        edges, nodes = create_manhattan_cartography(n_x=3, n_y=3)

        # All nodes should have type "N/A"
        assert all(nodes["type"] == "N/A")

    def test_edge_attributes(self):
        """Test that edges have correct attributes."""
        maxspeed = 60.0
        edges, nodes = create_manhattan_cartography(n_x=3, n_y=3, maxspeed=maxspeed)

        # Check nlanes
        assert all(edges["nlanes"] == 1)

        # Check type
        assert all(edges["type"] == "primary")

        # Check maxspeed
        assert all(edges["maxspeed"] == maxspeed)

        # Check that all edges have names
        assert all(edges["name"].str.startswith("grid_street_"))

    def test_edge_length_calculation(self):
        """Test that edge lengths are calculated correctly."""
        spacing = 1000.0  # 1 km
        edges, nodes = create_manhattan_cartography(n_x=3, n_y=3, spacing=spacing)

        # All edges should have approximately the same length (spacing)
        # Allow for small numerical errors
        assert all(abs(edges["length"] - spacing) < 1.0)

    def test_center_coordinates(self):
        """Test that the grid is centered at the specified coordinates."""
        center_lat, center_lon = 45.0, 10.0
        edges, nodes = create_manhattan_cartography(
            n_x=5, n_y=5, center_lat=center_lat, center_lon=center_lon
        )

        # Calculate the centroid of all nodes
        mean_lat = nodes.geometry.y.mean()
        mean_lon = nodes.geometry.x.mean()

        # Check that the centroid is close to the specified center
        # Allow for small numerical errors
        assert abs(mean_lat - center_lat) < 0.01
        assert abs(mean_lon - center_lon) < 0.01

    def test_geometry_types(self):
        """Test that geometries have the correct types."""
        edges, nodes = create_manhattan_cartography(n_x=3, n_y=3)

        # All nodes should have Point geometries
        assert all(nodes.geometry.geom_type == "Point")

        # All edges should have LineString geometries
        assert all(edges.geometry.geom_type == "LineString")

    def test_crs(self):
        """Test that GeoDataFrames have the correct CRS."""
        edges, nodes = create_manhattan_cartography(n_x=3, n_y=3)

        # Both should use EPSG:4326 (WGS 84)
        assert edges.crs == "EPSG:4326"
        assert nodes.crs == "EPSG:4326"

    def test_edge_connectivity(self):
        """Test that edges connect valid nodes."""
        edges, nodes = create_manhattan_cartography(n_x=4, n_y=4)

        node_ids = set(nodes["id"])

        # All edge sources and targets should be valid node IDs
        assert all(edges["source"].isin(node_ids))
        assert all(edges["target"].isin(node_ids))

    def test_no_self_loops(self):
        """Test that there are no self-loops (edges from a node to itself)."""
        edges, nodes = create_manhattan_cartography(n_x=5, n_y=5)

        # No edge should have the same source and target
        assert all(edges["source"] != edges["target"])

    def test_small_grid(self):
        """Test with a minimal grid size."""
        edges, nodes = create_manhattan_cartography(n_x=2, n_y=2)

        # 2x2 grid has 4 nodes
        assert len(nodes) == 4

        # 2x2 grid has 4 undirected edges, so 8 directed edges
        assert len(edges) == 8

    def test_custom_spacing(self):
        """Test with custom spacing."""
        spacing1 = 500.0
        spacing2 = 2000.0

        edges1, _ = create_manhattan_cartography(n_x=3, n_y=3, spacing=spacing1)
        edges2, _ = create_manhattan_cartography(n_x=3, n_y=3, spacing=spacing2)

        # Edges with larger spacing should have larger lengths
        assert edges1["length"].mean() < edges2["length"].mean()

    def test_rectangular_grid(self):
        """Test with a rectangular (non-square) grid."""
        n_x, n_y = 10, 3
        edges, nodes = create_manhattan_cartography(n_x=n_x, n_y=n_y)

        assert len(nodes) == n_x * n_y

        # Calculate expected edges
        expected_edges = 2 * (n_y * (n_x - 1) + n_x * (n_y - 1))
        assert len(edges) == expected_edges


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
