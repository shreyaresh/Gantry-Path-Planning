import math as math
import numpy as np


class Edge():
    """
    A weighted edge between nodes in the graph.

    Parameters:
    source = the beginning node of the Edge object, Node object
    end = the end node of the Edge object, Node object
    distance = the weight between two nodes
    """
    def __init__(self, source, end, distance):
        self._source = source
        self._end = end
        self._distance = distance
    @property
    def source(self):
        return self._source
    @property
    def end(self):
        return self._end
    @property
    def distance(self):
        return self._distance

class Graph():
    """
    Graph for all nodes and edges.
    """
    def __init__(self):
        self._nodes_list = set()
        self._edges_list = {}
        self.node_g_rhs = {}

    def add_node(self, coord):
        # Adds node to the list of all nodes
        self._nodes_list.add(coord)
        self.node_g_rhs[coord] = [float('inf'), float('inf')]
    
    def get_all_nodes(self):
        return self._nodes_list

    def add_edge(self, source, end):
        """
        Adds a bidirectional edge given two nodes.
        """
        # Adds nodes to graph if nodes are not in graph
        self.add_node(source)
        self.add_node(end)

        # Retrieve current edges
        source_edges = self._edges_list.get(source, set())
        end_edges = self._edges_list.get(end, set())
        
        # Calculating distance between two of the nodes
        x1, y1, z1 = source
        x2, y2, z2 = end

        weight = np.sqrt((x1-x2)**2 + (y1-y2)**2 + (z1-z2)**2)

        # Creating a bidirectional edge
        new_edge_from_source = Edge(source, end, weight)
        new_edge_from_end = Edge(end, source, weight)
        
        source_edges.add(new_edge_from_source)
        end_edges.add(new_edge_from_end)

        self._edges_list[source] = source_edges
        self._edges_list[end] = end_edges
    

    def get_neighbor_edges(self, node):
        # Retrieves edges coming from a node
        if not node in self._nodes_list:
            raise ValueError("Node is not present in the graph.")
        return self._edges_list.get(node, set())
