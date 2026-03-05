from datetime import datetime
import logging

from dsf.cartography import get_cartography
from dsf.mobility import RoadNetwork, Dynamics

from tqdm import trange
import numpy as np

SEED = 42
np.random.seed(SEED)

logging.basicConfig(level=logging.INFO)

if __name__ == "__main__":
    logging.info("Getting data from OpenStreetMap...")
    # Get the cartography of Modena, Italy
    G, df_edges, df_nodes = get_cartography("Parma, Emilia-Romagna, Italy")
    # set nlanes to 1 if 0
    df_edges["nlanes"] = df_edges["nlanes"].replace(0, 1).fillna(1).astype(int)

    df_edges.to_csv("modena_edges.csv", sep=";", index=False)
    df_nodes.to_csv("modena_nodes.csv", sep=";", index=False)

    nodes = G.nodes(data=False)
    # Extract 10% random node ids as origins and destinations for the traffic simulation
    origin_ids = np.random.choice(
        list(nodes), size=int(0.1 * len(nodes)), replace=False
    )
    destination_ids = np.random.choice(
        list(nodes), size=int(0.1 * len(nodes)), replace=False
    )

    del df_edges, df_nodes, G

    logging.info("Creating road network and dynamics model...")

    # Create a road network from the cartography
    road_network = RoadNetwork()
    road_network.importEdges("modena_edges.csv", ";")
    road_network.importNodeProperties("modena_nodes.csv", ";")
    # Adjust network parameters
    road_network.adjustNodeCapacities()
    road_network.autoMapStreetLanes()
    road_network.autoAssignRoadPriorities()
    road_network.autoInitTrafficLights()

    # Generaate a random vector of integer values for vehicle input
    # We want values to have a 10s entry for a whole day
    vehicle_input = np.random.randint(0, 10, size=8640)

    # Create a dynamics model for the road network
    dynamics = Dynamics(road_network, seed=SEED, alpha=0.8)
    # Get epoch time of today at midnight
    epoch_time = int(
        datetime.combine(datetime.today(), datetime.min.time()).timestamp()
    )

    dynamics.setInitTime(epoch_time)
    dynamics.connectDataBase("modena.db")
    dynamics.saveData(300, True, True, True)

    dynamics.setOriginNodes(origin_ids)
    dynamics.setDestinationNodes(destination_ids)

    # Simulate traffic for 24 hours with a time step of 10 seconds
    for time_step in trange(8640):
        # Update paths every 5 minutes (300 seconds)
        if time_step % 300 == 0:
            dynamics.updatePaths()
        # Add agents every 10 seconds
        if time_step % 10 == 0:
            dynamics.addAgentsRandomly(vehicle_input[time_step])

    dynamics.summary()
