#!/usr/bin/python3
import math

from CS312Graph import *
import time


class NetworkRoutingSolver:
    def __init__(self):
        self.network = None
        self.source = None
        self.distances = {}
        self.previous = {}

    def initializeNetwork(self, network):
        assert (type(network) == CS312Graph)
        self.network = network

    def getShortestPath(self, destIndex):
        # generate the path from the destination node to the source node using the previous nodes dictionary
        path = []
        node = self.network.nodes[destIndex]
        while node.node_id != self.source.node_id:
            path.append(node.node_id)
            if node.node_id not in self.previous:
                return {'cost': float('inf'), 'path': []}
            node = self.previous[node.node_id]
        path.append(self.source.node_id)
        path.reverse()
        # generate the path edges
        path_edges = []
        total_length = 0
        for i in range(len(path) - 1):
            for edge in self.network.nodes[path[i]].neighbors:
                if edge.dest.node_id == path[i + 1]:
                    path_edges.append((edge.src.loc, edge.dest.loc, '{:.0f}'.format(edge.length)))
                    total_length += edge.length
        return {'cost': total_length, 'path': path_edges}

    def computeShortestPaths(self, srcIndex, use_heap=False):
        self.source = self.network.nodes[srcIndex]
        self.previous = {}
        self.distances = {}
        t1 = time.time()
        if use_heap:
            self.dijkstra_heap(srcIndex)
        else:
            self.dijkstra_array(srcIndex)
        t2 = time.time()
        # set each node equal to its previous node
        for i in range(len(self.network.nodes)):
            for edge in self.network.nodes[i].neighbors:
                if self.distances[i] + edge.length == self.distances[edge.dest.node_id]:
                    # add node to previous dictionary
                    self.previous[edge.dest.node_id] = self.network.nodes[i]
        return t2 - t1

    # implement dijkstra's algorithm using an array
    def dijkstra_array(self, srcIndex):
        # initialize distances
        for i in range(len(self.network.nodes)):
            self.distances[i] = float('inf')
        self.distances[srcIndex] = 0
        # initialize visited
        visited = [False] * len(self.network.nodes)
        # initialize current node
        current = srcIndex
        # loop until all nodes have been visited
        while False in visited:
            # update distances
            for edge in self.network.nodes[current].neighbors:
                if self.distances[current] + edge.length < self.distances[edge.dest.node_id]:
                    self.distances[edge.dest.node_id] = self.distances[current] + edge.length
            # mark current node as visited
            visited[current] = True
            # find next node
            min_distance = float('inf')
            for i in range(len(self.distances)):
                if self.distances[i] < min_distance and not visited[i]:
                    min_distance = self.distances[i]
                    current = i
            if min_distance == float('inf'):
                break

    # implement dijkstra's algorithm using a heap
    # def dijkstra_heap(self, srcIndex):
        # initialize distances
        # for i in range(len(self.network.nodes)):
        #    self.distances[i] = float('inf')
        # self.distances[srcIndex] = 0
        # initialize visited
        # visited = [False] * len(self.network.nodes)
        # initialize heap
        # heap = [(0, srcIndex)]
        # loop until all nodes have been visited
        # while False in visited:
            # if len(heap) == 0:
            #    break
            # update distances
            # current_distance, current = heapq.heappop(heap)
            # if not visited[current]:
            #    visited[current] = True
            #    for edge in self.network.nodes[current].neighbors:
            #        if self.distances[current] + edge.length < self.distances[edge.dest.node_id]:
            #           self.distances[edge.dest.node_id] = self.distances[current] + edge.length
            #            heapq.heappush(heap, (self.distances[edge.dest.node_id], edge.dest.node_id))

    # implement dijkstra's algorithm using a heap
    def dijkstra_heap(self, srcIndex):
        # initialize distances
        for i in range(len(self.network.nodes)):
            self.distances[i] = float('inf')
        self.distances[srcIndex] = 0
        # initialize visited
        visited = [False] * len(self.network.nodes)
        # initialize heap
        myHeap = [(0, srcIndex)]
        # loop until all nodes have been visited
        while False in visited:
            if len(myHeap) == 0:
                break
            # update distances
            current_distance, current = self.myHeapPop(myHeap)
            if not visited[current]:
                visited[current] = True
                for edge in self.network.nodes[current].neighbors:
                    if self.distances[current] + edge.length < self.distances[edge.dest.node_id]:
                        self.distances[edge.dest.node_id] = self.distances[current] + edge.length
                        self.myHeapPush(myHeap, (self.distances[edge.dest.node_id], edge.dest.node_id))

    # implement the heap function to pop a heap from scratch
    def myHeapPop(self, heap):
        if len(heap) == 0:
            return None
        if len(heap) == 1:
            return heap.pop()
        heap_min = heap[0]
        heap[0] = heap.pop()
        self.myHeapify(heap, 0)
        return heap_min

    # implement the heap function to push a heap from scratch
    def myHeapPush(self, heap, item):
        heap.append(item)
        self.myHeapify(heap, len(heap) - 1)

    # implement the heap function to heapify a heap from scratch
    def myHeapify(self, heap, index):
        if index == 0:
            return
        parent_index = math.floor((index - 1) / 2)
        if heap[index][0] < heap[parent_index][0]:
            heap[index], heap[parent_index] = heap[parent_index], heap[index]
            self.myHeapify(heap, parent_index)
