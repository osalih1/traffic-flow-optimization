"""
Helper functions to process a graph
"""

import osmnx as ox
import networkx as nx
import geopandas
import numpy as np


def process_graph(G):
    """
    Process a basic graph by simplifying and consolidating intersections

    Args:
        G: A networkx.MultiDiGraph generated with simplify=False

    Returns a networkx.MultiDiGraph that has been processed
    """
    G = ox.simplify_graph(G)
    G_proj = ox.project_graph(G)
    G2 = ox.consolidate_intersections(G_proj, rebuild_graph=True, dead_ends=False)

    return G2


def remove_duplicate_edge(G):
    """ """
    for u, v, ind in G.edges:
        if ind > 0:
            G.remove_edge(u, v, ind)


def add_speeds(G):
    """
    Add speeds in kph to a processed graph

    This finds average speeds for each road (edge) and, if not found, uses the highway type to
    replace the empty field with the average road speed for the entire map. Also adds edge travel
    time based on speed and distance

    Args:
        G: A networkx.MultiDiGraph that has already been processed

    Returns a networkx.MultiDiGraph that has speeds added to the 'speed_kph' column
    """
    edges = ox.graph_to_gdfs(G, nodes=False)
    edges["highway"] = edges["highway"].astype("str")

    hwy_speed = {
        "residential": 40,
        "primary": 85,
        "secondary": 48,
        "tertiary": 47,
        "trunk": 85,
        "motorway": 88.5,
        "unclassified": 45,
    }
    G = ox.add_edge_speeds(G, hwy_speeds=hwy_speed)
    G = ox.add_edge_travel_times(G)
    return G


def add_lanes(edges):
    """
    Add lanes to a processed graph

    This assumes 2 lanes for any road without total lanes given

    Args:
        edges: A geopandas.GeoDataFrame created from a graph that has already been processed

    Returns the same 'edges' that has total lanes added to the 'lanes' column
    """
    if "lanes" not in edges.columns:
        edges["lanes"] = 2

    elif edges["lanes"].isnull().sum() > 0:
        edges["lanes"] = edges["lanes"].fillna(2)

    return edges


def add_capacities(edges):
    """
    Add capacities (vehicles/hour) to a processed graph that has speeds and lanes

    Based on https://www.fhwa.dot.gov/policyinformation/pubs/pl18003/hpms_cap.pdf, the capacity
    calculation looks different for different types of roads. We use their estimation for highway
    speed and https://www.globalsecurity.org/military/library/policy/army/fm/19-25/CH25.htm#img103
    for other types of roads

    Provided here is a capacity generalization from HPMS road classification mapped to OSMNX's road
    classification system

        Motorways: Capacity = (2200 + (10 * (speed (mph) - 50)))) * lanes
            (motorway, primary)

        Multilane: Capacity = (1000 + (20 * speed (mph)) if speed < 60 else 2200) * lanes
            Assume that we can ignore heavy vehicle traffic
            (If lanes > 6)

        Signalled: Capacity = 0.5 * lanes * 1900
            Assume 50% green time and all secondary have signals
            (secondary)

        Stop-controlled: Capacity = 1200 if lanes = 2 else 1500
            (tertiary, unclassified, residential)

        Other roads: Assume to be signalled

    Args:
        edges: A geopandas.GeoDataFrame that has already been processed and has speeds and lanes
        added

    Returns the same 'edges' that has hourly capacity added to the 'capacity' column
    """
    edges["capacity"] = np.nan

    # Iterate through rows and add capacity
    for idx, row in edges.iterrows():
        speed_mph = float(row["speed_kph"]) * 0.621371

        # Process so that the minimum number of lanes is chosen if multiple number of lanes
        if "[" in str(row["lanes"]):
            lanes = min([int(i) for i in row["lanes"]])
            edges.at[idx, "lanes"] = lanes
        else:
            lanes = int(row["lanes"])

        # Process so that roads with two or more labels only use the first one
        if "[" in str(row["highway"]):
            row["highway"] = row["highway"][0]
            edges.at[idx, "highway"] = row["highway"]

        this_capacity = 0

        # First level of priority is capacity for typical highway
        if row["highway"] == "motorway" or row["highway"] == "primary":
            this_capacity = (2200 + (10 * (speed_mph - 50))) * int(lanes)

        # Second level of priority is number of lanes
        elif lanes > 6:
            if speed_mph < 60:
                this_capacity = (1000 + (20 * speed_mph)) * lanes
            else:
                this_capacity = 2200 * lanes

        # Third level of priority is stop-controlled
        elif row["highway"] in ["tertiary", "unclassified", "residential"]:
            if lanes <= 2:
                this_capacity = 1200

            else:
                this_capacity = 1500

        # Assume all other roads are signalled
        elif lanes <= 4:
            this_capacity = 0.5 * lanes * 1900
        else:
            this_capacity = 3800

        edges.at[idx, "capacity"] = this_capacity

    return edges


def add_attributes_to_graph(G, edges, attributes):
    """
    Add edges columns specified in attributes to the graph itself

    Args:
        G: A networkx.MultiDiGraph to add attributes to
        edges: A geopandas.GeoDataFrame including the new columns to be added to the graph
        attributes: A list of strings representing the column names from edges to be added to the
            graph

    Returns: G, the networkx.MultiDiGraph with the columns added to edges
    """
    for a in attributes:
        nx.set_edge_attributes(G, edges[a], a)

    return G
