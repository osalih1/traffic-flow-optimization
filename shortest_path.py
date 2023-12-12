"""
Module used to compute the shortest path and maximum flow from location A to location B.
"""

import osmnx as ox
import networkx as nx
import geopandas as gpd
from geopy.geocoders import Nominatim
from shapely.geometry import Point
import graph_processing as gp
from networkx.algorithms.flow import edmonds_karp


class ShortestPath:
    """
    Class used to compute the shortest path from location to location using
    Dijkstra's and A* as well as find maximum flow and minimum cut using Edmonds-Karp

    Attributes:
        start_location: String representing the starting location. Can be a town, or street address
        end_location: String representing the end location. Can be a town, or street address.
    """

    start_location = ""
    end_location = ""

    def __init__(self, input_addresses, set_radius=1500, inv_capacity=False):
        """
        Attributes:
            input_addresses: A list of strings representing a list of addresses.
            set_radius: An int representing the radius to set the graph to based off of the first
                address, default is 1500m
            inv_capacity: A boolean representing whether an additional column with inverse capacity
                should be added to the graph. This is a testing purpose metric and does not
                necessarily correspond to real life situations. Default is false
        """
        self.coordinate_dict = {}
        self.radius = set_radius
        print("Converting addresses...")
        self.address_to_coords(input_addresses)
        print("Creating graph...")
        self.graph = self.create_graph(
            self.coordinate_dict[input_addresses[0]], self.radius, inv_capacity=inv_capacity
        )
        print("Graph created & setup complete.")

    def create_graph(self, coords, radius=1500, inv_capacity=False):
        """
        Creates a graph of the local area.

        Graph is created from the coordinates inputted including just drivable roads. This is then
        processed based on the graph_processing library's process_graph function (simplified,
        projected, intersections consolidated). Speeds, lanes, and capacities are added to the graph

        Args:
            coords: Tuple representing the longitude, latitude coordinates of desired area.
            radius: An integer representing the radius of the target circle in meters, with the
                center being `coords`. Default is 1500m.
            inv_capacity: A boolean representing whether an additional column with inverse capacity
                should be added to the graph. This is a testing purpose metric and does not
                necessarily correspond to real life situations. Default is false

        Returns:
        An OSMnx graph object representing a graph of the desired area that has been processed
        """
        G = ox.graph_from_point(coords, network_type="drive", dist=radius, simplify=False)
        G = gp.process_graph(G)
        G = gp.add_speeds(G)
        edges = ox.graph_to_gdfs(G, nodes=False)
        edges = gp.add_lanes(edges)
        edges = gp.add_capacities(edges, inv_capacity=inv_capacity)
        G = gp.add_attributes_to_graph(G, edges, ["capacity", "speed_kph", "lanes", "highway"])
        if inv_capacity:
            G = gp.add_attributes_to_graph(G, edges, ["inv_capacity"])
        return G

    def address_to_coords(self, addresses):
        """
        Converts an address to longitude and latitude.

        Args:
            addresses: List of strings representing locations. Can be towns, or street addresses.

        Returns:
        A tuple representing a coordinate (latitude, longitude)
        """
        # Call the Nominatim tool and create Nominatim class
        location = Nominatim(user_agent="Geopy Library")

        if isinstance(addresses, str):
            addresses = [addresses]

        # Enter the location name to coordinate_dict
        for address in addresses:
            print(f"Adding '{address}' to dictionary.")
            get_location = location.geocode(address)
            self.coordinate_dict[address] = (get_location.latitude, get_location.longitude)

        return (get_location.latitude, get_location.longitude)

    def nearest_node(self, location):
        """
        Finds the nearest node to an address in an OSMnx graph object.

        Args:
            location: A string representing the geographical address of a
                specified location.

        Returns:
        Nearest node IDs or optionally a tuple where dist contains distances between the points and
        their nearest nodes
        """
        # Check if the coordinates exist in dict, and if not, converts them.
        coords = self.check_dictionary(location)
        lat = coords[0]
        long = coords[1]

        # Convert coordinates to the 2D space
        point = Point((long, lat))
        crs_point = gpd.GeoSeries(point, crs="epsg:4326")

        # Project the point to the same CRS as the projected graph
        points_proj = crs_point.to_crs(self.graph.graph["crs"])

        return ox.distance.nearest_nodes(self.graph, points_proj.x, points_proj.y)

    def check_dictionary(self, location):
        """
        Checks if the target location is in the coordinate dictionary, and if
        not, adds it to the dictionary.

        Args:
            location: A string representing a street address.

        Returns: Coordinates as a tuple of (latitude, longitude) stored in the dictionary for the
        location inputted
        """
        if location in self.coordinate_dict:
            coords = self.coordinate_dict[location]
        else:
            coords = self.address_to_coords(location)
        return coords

    def d_shortest_path(self, start, end, weight="length"):
        """
        Renders shortest path from location A to location B using NetworkX's implementation of
        Dijkstra's

        Args:
            start: A integer representing the node ID of the start location.
            end: A integer representing the node ID of the end location.
            weight: A string representing a column to use as weight for the shortest path. Default
                is length (distance of path)

        Returns:
        A list of node IDs representing the shortest path based on dijkstra's from the node closest
        to the starting location, to the node closest to the ending location, assuming that the
        edges of the graph are "below capacity", or don't have traffic.
        """
        # Compute the start and end node IDs.
        start_node = self.nearest_node(start)[0]
        end_node = self.nearest_node(end)[0]

        return nx.dijkstra_path(self.graph, start_node, end_node, weight=weight)

    def astar_shortest_path(self, start, end, weight="length"):
        """
        Discovers shortest path from location A to location B using NetworkX's implementation of A*.

        Args:
            start: A string representing the geographical address of the start location.
            end: A string representing the geographical address of the end location.
            weight: A string representing a column to use as weight for the shortest path. Default
                is length (distance of path)

        Returns:
        A list of node IDs representing the shortest path based on A* from the node closest to the
        starting location, to the node closest to the ending location, assuming that the edges of
        the graph are "below capacity", or don't have traffic.
        """
        # Computes the start and end node IDs.
        start_node = self.nearest_node(start)[0]
        end_node = self.nearest_node(end)[0]

        return nx.astar_path(self.graph, start_node, end_node, heuristic=None, weight=weight)

    def max_flow_path(self, start, end):
        """
        Discover shortest path from location A to location B based on capacities assuming no
        edge is over capacity based on Edmond's-Karp

        Args:
            start: A string representing the geographical address of the start location.
            end: A string representing the geographical address of the end location.

        Returns:
        flow_value representing the value of the maximum flow (total cars that can take the shortest
        path by capacity) and flow_dict representing the path of nodes taken by the optimal path.
        """
        # Compute the start and end node IDs.
        start_node = self.nearest_node(start)[0]
        end_node = self.nearest_node(end)[0]

        # Find max flow of graph
        flow_value, flow_dict = nx.maximum_flow(
            nx.Graph(self.graph), start_node, end_node, capacity="capacity", flow_func=edmonds_karp
        )

        # Add "actual_flow" attribute to graph edges. This is set to 0 if edge is not selected for
        # max flow
        path = {
            (u, v, 0): {"actual_flow": flow_dict[u][v]}
            for u in flow_dict
            for v in flow_dict[u]
            if flow_dict[u][v] > 0
        }
        nx.set_edge_attributes(self.graph, path)

        return flow_value, flow_dict

    def min_cut(self, start, end):
        """
        Find the min cut for the graph from location A to location B based on capacities based on
        Edmonds-Karp

        Args:
            start: A string representing the geographical address of the start location.
            end: A string representing the geographical address of the end location.

        Returns:
        cut_value representing the min_cut of the graph (maximum cars that can go from point A to B)
        partition (node sets present in two sections of graph), and cutset representing sets of 2
        edges that make up the min cut
        """
        # Computes the start and end node IDs.
        start_node = self.nearest_node(start)[0]
        end_node = self.nearest_node(end)[0]

        # Find min cut and partition of graph into two sections
        cut_value, partition = nx.minimum_cut(
            nx.Graph(self.graph), start_node, end_node, capacity="capacity", flow_func=edmonds_karp
        )

        # Find edges making up the min cut of the graph
        cutset = nx.minimum_edge_cut(
            nx.Graph(self.graph), start_node, end_node, flow_func=edmonds_karp
        )

        return cut_value, partition, cutset
