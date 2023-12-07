import osmnx as ox
import networkx as nx
from geopy.geocoders import Nominatim
from shortest_path import ShortestPath

address_list = ["1000 Olin Way, Needham, MA", "958 Highland Ave, Needham, MA", "Amherst, MA"]
alg = ShortestPath(address_list, 5000)
# print(alg.coordinate_dict)
olin = address_list[0]
tjs = address_list[1]
medford = address_list[2]
print(alg.coordinate_dict[medford])
print(alg.coordinate_dict[olin])
print(alg.coordinate_dict[tjs])

# Attempting to project the graph
locations = [[alg.coordinate_dict[medford][0] , alg.coordinate_dict[olin][0], alg.coordinate_dict[tjs][0]],[alg.coordinate_dict[medford][1] , alg.coordinate_dict[olin][1], alg.coordinate_dict[tjs][1]] ]

# P = ox.projection.project_graph(alg.graph)
print(ox.distance.nearest_nodes(alg.graph, locations[1], locations[0]))

# print(alg.nearest_node(olin))
# print(alg.nearest_node(tjs))
# print(alg.nearest_node(amherst))
# shortest_path_list = alg.d_shortest_path(address_list[0], address_list[1])
# print(shortest_path_list)
# nc = ["y" if () else "r" for node in alg.graph.nodes()]
# fig, ax = ox.plot_graph(alg.graph, node_color="r")
