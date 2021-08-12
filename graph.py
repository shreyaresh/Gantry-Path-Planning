import math as math
import numpy as np


class Node():
    """
    A singular node in the graph.

    Parameters:
    coord = tuple of x, y, and z coordinates
    """
    def __init__(self, coord):
        self.coord = coord
    def __repr__(self):
        return repr("Node", coord)


class Edge():
    """
    A weighted edge between nodes in the graph.
    """
    def __init__(self, source, end, distance):
        self.source = source
        self.end = end
        self.distance = distance
    @property
    def source(self, source):
        return self._source
    @property
    def end(self, end):
        return self._end
    @property
    def distance(self, distance):
        return self._distance

class Graph():
    """
    Graph for all nodes and edges.
    """
    def __init__(self):
        self.nodes_list = set()
        self.edges_list = {}

    def add_node(self, node):
        self.nodes_list.add(node)
    
    def add_edge(self, source, end, weight=0.0):
        # adds nodes to graph if nodes are not in graph
        self.add_node(source)
        self.add_node(end)
        source_edges = self.edges_list.get(source, set())
        end_edges = self.edges_list.get(end, set())
        
        # creating the edge from both directions
        new_edge_from_source = Edge(source, end, weight)
        new_edge_from_end = Edge(end, source, weight)
        
        source_edges.add(new_edge_from_source)
        end_edges.add(new_edge_from_end)

        self.edges_list[source] = source_edges
        self.edges_list[end] = end_edges

if __name__ == __main__:
    new_node = Node((1,2,3))
    print(new_node)