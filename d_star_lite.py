import grid

"""
File containing the D* Lite algorithm.
Missing: live replanning of new obstacles, not necessary because we operate in a known environment and the
graph does not contain obstacles.
"""

def heuristic(coord1, coord2):
    x =  abs(coord1[0] - coord2[0])
    y =  abs(coord1[1] - coord2[1])
    z =  abs(coord1[2] - coord2[2])
    return max(x,y,z)

class Priority():
    """
    Stored as a list of [..., (key, backup_key, (coordinate)), ...]
    
    """
    def __init__(self):
        self.queue = []
    
    def insert(self, s, k):
        new = k + (s,)
        self.queue.append(new)

    def top(self):
        self.queue.sort(key=lambda tup: tup[0])
        return self.queue[0][2]

    def topKey(self):
        if len(self.queue) == 0:
            return (float('inf'), float('inf'))
        self.queue.sort(key=lambda tup: tup[0])
        return self.queue[0][:2]

    def pop(self):
        self.queue.sort(key=lambda tup: tup[0])
        # tiebreaker, use second key
        if len(self.queue) > 1:
            if self.queue[0][0] == self.queue[1][0]:
                smaller_queue = [self.queue[i] for i in range(len(self.queue)) if self.queue[i][0] == self.queue[0][0]]
                smaller_queue.sort(key=lambda tup: tup[1])
                self.queue.remove(smaller_queue[0])
                return smaller_queue[0][2]

        popped = self.queue.pop(0)
        return popped[2]

    def update(self, s, k):
        idx = 0
        for i, el in enumerate(self.queue):
            if el[2] == s:
                idx = i
                break
        if self.queue[idx][0:1] != k:
            self.queue[idx][0] = k[0]
            self.queue[idx][0] = k[1]

    def remove(self, s):
        for i, el in enumerate(self.queue):
            if el[2] == s:
                idx = i
                break
        self.queue.remove(self.queue[idx])
    
    def __repr__(self):
        return str(self.queue)

class DStarLite():

    def __init__(self, graph, start, end):
        self.graph = graph
        self._start = start
        self._end = end
        self.U = Priority()

    ####################
    # HELPER FUNCTIONS #
    ####################

    def set_g(self, vertex, val):
        self.graph.node_g_rhs[vertex][0] = val
    
    def set_rhs(self, vertex, val):
        self.graph.node_g_rhs[vertex][1] = val

    def g(self, vertex):
        return self.graph.node_g_rhs[vertex][0]
    
    def rhs(self, vertex):
        return self.graph.node_g_rhs[vertex][1]

    def calculateKey(self, vertex):
        return (min(self.g(vertex), self.rhs(vertex)) + heuristic(vertex, self._start), min(self.g(vertex), self.rhs(vertex)))

    def initialize(self):
        self.set_rhs(self._end, 0)
        self.U.insert(self._end, self.calculateKey(self._end))

    def updateVertex(self, u):
        if u != self._end:
            min_rhs = float('inf')
            for i in self.graph.get_neighbor_edges(u):
                node = i.end
                min_rhs = min(min_rhs, self.g(node) + i.distance)
            self.set_rhs(u, min_rhs)

        coord_queue = [node for node in self.U.queue if u == node[2]]
        if coord_queue != []:
            self.U.remove(u)
        if self.g(u) != self.rhs(u):
            self.U.insert(u, self.calculateKey(u))


    def computeShortestPath(self):
        while self.U.topKey() < self.calculateKey(self._start) or self.rhs(self._start) != self.g(self._start):
            k_old = self.U.topKey()
            u = self.U.pop()
            if k_old < self.calculateKey(u):
                self.U.insert(u, self.calculateKey(u))
            elif self.g(u) > self.rhs(u):
                self.set_g(u, self.rhs(u))
                for s in list(self.graph.get_neighbor_edges(u)):
                    # potential problem spot
                    self.updateVertex(s.end)
            else:
                self.set_g(u, float('inf'))
                for s in self.graph.get_neighbor_edges(u):
                    self.updateVertex(s.end)
                self.updateVertex(u)

    ###################
    #       MAIN      #
    ###################

    def main(self):
        self.initialize()
        self.computeShortestPath()
        path = []
        curr = self._start
        while curr != self._end:
            path.append(curr)
            min_rhs = float('inf')
            for child in self.graph.get_neighbor_edges(curr):
                cost = self.g(child.end) + child.distance
            
                if cost < min_rhs:
                    min_rhs = cost
                    curr = child.end
        path.append(self._end)
        return path
