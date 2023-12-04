import osmnx as ox
import networkx as nx
from geopy.geocoders import Nominatim
from shortest_path import ShortestPath

address_list = ["1000 Olin Way, Needham, MA", "958 Highland Ave, Needham, MA", "Medford, MA"]
alg = ShortestPath(address_list, 5000)
# print(alg.coordinate_dict)
olin = address_list[0]
tjs = address_list[1]
print(alg.coordinate_dict[olin])
print(alg.coordinate_dict[tjs])
print(alg.nearest_node(alg.graph, olin))
print(alg.nearest_node(alg.graph, tjs))
shortest_path_list = alg.d_shortest_path(alg.graph, address_list[0], address_list[1])
print(shortest_path_list)
# fig, ax = ox.plot_graph(alg.graph, node_color="r")

if __name__ == '__main__':
    main()