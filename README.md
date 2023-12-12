# traffic-flow-optimization

Final project for FA'23 Discrete Math
Traffic Optimization Analysis through shortest path and maximum flow in Python

## Overview

We aim to analyze traffic flow in Python by utilizing OSMnx and NetworkX's implementations of the two shortest path algorithms (Dijkstra's and A*) with respect to time and distance as well as maximum flow through Edmond's-Karp to compare them. 

## File Structure

- `graph_processing.py`: contains a graph processing library. Preprocesses the initial graph and adds attributes such as capacity, speed, lanes necessary to analyze it
- `shortest_path.py`: contains an implementation of the ShortestPath class. Contains functions to find different types of shortest paths, maximum flow, and minimum cut for the graph.
- `main.ipynb`: contains a walkthrough of the traffic analysis done for the project. Includes visualizations of various paths.
- `requirements.txt`: contains the required dependencies needed to run the Python scripts and Jupyter notebooks. 

## Usage
- Clone the repo and make sure the files specified above are present
- Navigate to the repository and install the requirements specified in `requirements.txt` by running `pip install -r requirements.txt`. This project was specified for specific library version which are contained in the requirements file, and using a virtual environment may be more convenient.
- Walk through the Jupyter Notebook to gain a basic understanding of how the implementation of shortest path and maximum flow has been done.
