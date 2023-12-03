"""
Module used to compute the shortest path from location to location.
"""

import osmnx as ox
import networkx as nx
from geopy.geocoders import Nominatim


class ShortestPath():
    """
    Class used to compute the shortest path from location to location using
    Dijkstra's and A*.

    Attributes:
        start_location: String representing the starting location. Can be a
            town, or street address.
        end_location: String representing the end location. Can be a town, or
            street address.
    """
    start_location = ""
    end_location = ""

    def __init__(self):
        """
        TODO: Assign instance attributes and create docstring.
        """

    def create_graph(self, coords, radius=1500):
        """
        Creates a graph of the local area.

        Args:
            coords: Tuple representing the longitude, latitude coordinates of
                desired area.
            radius: An integer representing the radius of the target circle in
                meters, with the center being `coords`. Default is 1500m.

        Returns:
            An OSMnx graph object representing a graph of the desired area.
        """
        osmnx_graph = ox.graph_from_point(coords, network_type="drive", dist=radius)
        # Displays the graph using an OSMnx plot
        #* fig, ax = ox.plot_graph(G, node_color="r")
        return osmnx_graph

    def address_to_coords(self, address="Needham, MA"):
        """
        Converts an address to longitude and latitude.

        Args:
            address: String representing a location. Can be a town, or street
                address.

        Returns:
            A tuple representing a coordinate (latitude, longitude)
        """
        # * calling the Nominatim tool and create Nominatim class
        location = Nominatim(user_agent="Geopy Library")

        # * entering the location name
        get_location = location.geocode(address)

        # * printing address
        # print(get_location.address)

        # * printing latitude and longitude
        # print("Latitude = ", get_location.latitude, "\n")
        # print("Longitude = ", get_location.longitude)

        return (get_location.latitude, get_location.longitude)

    def nearest_node(self, graph, location):
        """
        Finds the nearest node to an address in an OSMnx graph object.

        Args:
            graph: An OSMnx graph object representing a graph of a select area.
            location: A string representing the geographical address of a
                specified location.

        Returns:
            Nearest node IDs or optionally a tuple where dist contains distances
            between the points and their nearest nodes
        """
        coords = self.address_to_coords(location)
        # ? Should an error calculation between node and location occur?
        return ox.distance.nearest_nodes(graph, coords[0], coords[1])

    def d_shortest_path(self, graph, start="1000 Olin Way, Needham, MA",
                        end="958 Highland Ave, Needham, MA"):
        """
        Renders shortest path from location A to location B using Dijkstra's

        Args:

        graph: An OSMnx graph object representing a graph of the desired area.
        start: A string representing the geographical address of the start
            location.
        end: A string representing the geographical address of the end location.

        Returns:
            The shortest path from the node closest to the starting location, to
            the node closest to the ending location, assuming that the edges of
            the graph are "below capacity", or don't have traffic.
        """
        return ox.routing.shortest_path(graph, start, end)

    def astar_shortest_path(self, graph, start="1000 Olin Way, Needham, MA",
                            end="958 Highland Ave, Needham, MA"):
        """
        Discovers shortest path from location A to location B using A*.

        Args:

        graph: An OSMnx graph object representing a graph of the desired area.

        start: A string representing the geographical address of the start
            location.

        end: A string representing the geographical address of the end location.

        Returns:
            A list of node IDs representing the shortest path from the node
            closest to the starting location, to the node closest to the ending
            location, assuming that the edges of the graph are "below capacity",
            or don't have traffic.
        """
        #! Must convert the input graph from OSMnx to NetworkX, and then use
        #! the nx astar function to compute the shortest path using A*.
