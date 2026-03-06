import argparse
from datetime import datetime
import logging

from dsf.cartography import get_cartography
from dsf.mobility import RoadNetwork, Dynamics

from tqdm import trange
import numpy as np
import networkx as nx

logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--seed", type=int, default=69, help="Random seed for reproducibility"
    )
    parser.add_argument(
        "--city", type=str, required=True, help="City name for cartography"
    )
    parser.add_argument(
        "--country", type=str, default="Italy", help="Country name for cartography"
    )
    args = parser.parse_args()
    np.random.seed(args.seed)
    args.city = args.city.lower().strip().replace(" ", "_")
    args.country = args.country.lower().strip().replace(" ", "_")

    logging.info(
        f"Getting data from OpenStreetMap for {args.city.capitalize()}, {args.country.capitalize()}..."
    )
    # Get the cartography of the specified city
    G, df_edges, df_nodes = get_cartography(
        {"city": args.city.capitalize(), "country": args.country.capitalize()},
        infer_speeds=True,
    )

    df_edges.to_csv(f"{args.city}_{args.country}_edges.csv", sep=";", index=False)
    df_nodes.to_csv(f"{args.city}_{args.country}_nodes.csv", sep=";", index=False)

    # Keep only strong connected components
    G = G.subgraph(max(nx.strongly_connected_components(G), key=len)).copy()
    nodes = G.nodes(data=False)
    # Extract 10% random node ids as origins and destinations for the traffic simulation
    origin_ids = np.random.choice(
        list(nodes), size=int(0.05 * len(nodes)), replace=False
    )
    destination_ids = np.random.choice(
        list(nodes), size=int(0.05 * len(nodes)), replace=False
    )

    origins = {node_id: 1 for node_id in origin_ids}
    destinations = {node_id: 1 for node_id in destination_ids}

    del df_edges, df_nodes, G

    logging.info("Creating road network and dynamics model...")

    # Create a road network from the cartography
    road_network = RoadNetwork()
    road_network.importEdges(f"{args.city}_{args.country}_edges.csv", ";")
    road_network.importNodeProperties(f"{args.city}_{args.country}_nodes.csv", ";")
    # Adjust network parameters
    road_network.adjustNodeCapacities()
    road_network.autoMapStreetLanes()
    road_network.autoAssignRoadPriorities()
    road_network.autoInitTrafficLights()
    road_network.describe()

    # Generaate a random vector of integer values for vehicle input
    # We want values to have a 10s entry for a whole day
    vehicle_input = np.random.randint(0, 50, size=8640)

    # Create a dynamics model for the road network
    dynamics = Dynamics(road_network, seed=args.seed, alpha=0.8)
    # Get epoch time of today at midnight
    epoch_time = int(
        datetime.combine(datetime.today(), datetime.min.time()).timestamp()
    )

    dynamics.killStagnantAgents(40.0)
    dynamics.setInitTime(epoch_time)
    dynamics.connectDataBase(f"{args.city}_{args.country}.db")
    dynamics.saveData(300, True, True, True)

    dynamics.setOriginNodes(origins)
    dynamics.setDestinationNodes(destinations)

    # Simulate traffic for 24 hours with a time step of 10 seconds
    for time_step in trange(86400):
        # Update paths every 5 minutes (300 seconds)
        if time_step % 300 == 0:
            dynamics.updatePaths()
        # Add agents every 10 seconds
        if time_step % 10 == 0:
            dynamics.addAgentsRandomly(vehicle_input[time_step // 10])
        dynamics.evolve(False)

    dynamics.summary()
