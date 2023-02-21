#!/usr/bin/python3
import heapq

from CS312Graph import *
import time


class NetworkRoutingSolver:
    def __init__(self):
        self.source = None
        self.distancesHeap = {}
        self.distancesArray = {}

    def initializeNetwork(self, network):
        assert (type(network) == CS312Graph)
        self.network = network

    def getShortestPath(self, destIndex):
        self.dest = destIndex
        # TODO: RETURN THE SHORTEST PATH FOR destIndex
        #       INSTEAD OF THE DUMMY SET OF EDGES BELOW
        #       IT'S JUST AN EXAMPLE OF THE FORMAT YOU'LL 
        #       NEED TO USE
        path_edges = []
        total_length = 0
        node = self.network.nodes[self.source]
        edges_left = 3
        while edges_left > 0:
            edge = node.neighbors[2]
            path_edges.append((edge.src.loc, edge.dest.loc, '{:.0f}'.format(edge.length)))
            total_length += edge.length
            node = edge.dest
            edges_left -= 1
        return {'cost': total_length, 'path': path_edges}

    def computeShortestPaths(self, srcIndex, use_heap=False):
        self.source = srcIndex
        t1 = time.time()
        # TODO: RUN DIJKSTRA'S TO DETERMINE SHORTEST PATHS.
        #       ALSO, STORE THE RESULTS FOR THE SUBSEQUENT
        #       CALL TO getShortestPath(dest_index)
        if use_heap:
            self.dijkstra_heap(srcIndex)
        else:
            self.dijkstra_array(srcIndex)
        t2 = time.time()
        return t2 - t1

    def dijkstra_heap(self, srcIndex):
        distances = {}  # dictionary to keep track of the shortest distance to each node
        for node in self.network.nodes:
            distances[node] = float('inf')  # set all distances to infinity initially
        distances[srcIndex] = 0  # set the distance to the starting node to 0

        heap = [(0, srcIndex)]  # create a binary heap with the starting node and its distance

        while heap:
            (dist, current_node) = heapq.heappop(heap)  # get the node with the smallest distance
            if dist > distances[current_node]:
                continue  # if the distance is larger than the recorded distance, ignore it
            for neighbor, weight in self.network[current_node].items():
                distance = dist + weight
                if distance < distances[neighbor]:
                    distances[neighbor] = distance  # update the distance to the neighbor
                    heapq.heappush(heap, (distance, neighbor))  # add the neighbor to the binary heap

        self.distancesHeap = distances

    def dijkstra_array(self, srcIndex):
        distances = {}  # dictionary to keep track of the shortest distance to each node
        for node in self.network.nodes:
            distances[node] = float('inf')  # set all distances to infinity initially
        distances[srcIndex] = 0  # set the distance to the starting node to 0

        unvisited = [(node, distance) for node, distance in distances.items()]  # list of unvisited nodes

        while unvisited:
            current_node, current_distance = min(unvisited, key=lambda x: x[
                1])  # get the unvisited node with the smallest distance
            unvisited.remove((current_node, current_distance))
            for neighbor, weight in self.network[current_node].items():
                distance = distances[current_node] + weight
                if distance < distances[neighbor]:
                    distances[neighbor] = distance  # update the distance to the neighbor
                    unvisited.append((neighbor, distance))  # add the neighbor to the list of unvisited nodes

        self.distancesArray = distances
