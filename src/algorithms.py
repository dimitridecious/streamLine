import time
import heapq


class PathfindingAlgorithms:
    def __init__(self, graph=None):
        # init with graph
        self.graph = graph

    def set_graph(self, graph):
        # set new graph
        self.graph = graph

    def dijkstra(self, origin, dest):
        # bug fix
        if self.graph is None:
            raise ValueError("Graph not set.")

        start_time = time.time()
        # dist to each node
        distances = {node: float('inf') for node in self.graph.nodes}
        distances[origin] = 0
        # track the path
        prev = {origin: None}
        # min-heap of (dist, node) priority queues
        pq = [(0, origin)]
        checked = set()

        while pq:
            curr_dist, curr = heapq.heappop(pq)
                if curr in checked:
                    continue
                checked.add(curr)
            # check if dest reached, stop if true
            if curr == dest:
                break
            for i in self.graph.neighbors(curr):
                if i in checked:
                    continue
        # get edge length, handle multigraph structure
        try:
            edge_length = self.graph.edges[curr,i, 0]['length']
            except (KeyError, IndexError):
                edge_length = self.graph.edges[curr, i]['length']

        new_dist = curr_dist + edge_length

        # if shorter path, update structures
        if new_dist < distances.get(i, float('inf')):
        distances[i] = new_dist
        prev[i] = curr
    heapq.heappush(pq, (new_dist, i))

   if distances.get(dest, float('inf')) == float('inf'):
    # no path
   computation_time = time.time() - start_time
   return None, float('inf'), computation_time

    # backtracking from dest
   path = []
   node = dest
   while node is not None:
        path.append(node)
        node = prev.get(node)
        path.reverse()  # reverse to get origin->dest

    distance = distances[dest]
    computation_time = time.time() - start_time

    return path, distance, computation_time

    def greedy_best_first_search(self, origin, dest):
        # implement gbfs here using heuristic
        # bug fix
        if self.graph is None:
            raise ValueError("Graph not set.")

        start_time = time.time()
        # get coordinates for the heuristic calculation later
        origin_coords = (
            self.graph.nodes[origin]['y'], self.graph.nodes[origin]['x'])
        dest_coords = (
            self.graph.nodes[dest]['y'], self.graph.nodes[dest]['x'])
        # priority queue by heuristic value
        pq = [(self.euclidian_distance(origin_coords, dest_coords), origin)]
        heapq.heapify(pq)
        prev = {origin: None}
        checked = set()

        while pq:
            _, curr = heapq.heappop(pq)
            if curr == dest:
                break
            if curr in checked:
                continue
            checked.add(curr)
            for i in self.graph.neighbors(curr):
                if i not in checked:
                    prev[i] = curr
                    i_coords = (
                        self.graph.nodes[i]['y'], self.graph.nodes[i]['x'])
                    priority = self.euclidian_distance(
                        i_coords, dest_coords)
                    heapq.heappush(pq, (priority, i))

        # check for no route
        if dest not in prev:
            return None, float('inf'), time.time() - start_time

        # reconstruct path
        path = []
        curr = dest
        while curr is not None:
            path.append(curr)
            curr = prev[curr]
        path.reverse()

        # calculate the total distance by summing edge lengths
        distance = 0
        for i in range(len(path) - 1):
            distance += self.graph.edges[path[i], path[i+1], 0]['length']

        time_taken = time.time() - start_time
        return path, distance, time_taken

    def euclidian_distance(self, p1, p2):
        # euclidian distance
        return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5

    def algorithm_difference(self, origin, dest):
        # here compare dijkstras and gbfs
        # bug fix
        if self.graph is None:
            raise ValueError("Graph not set.")

        # run both algos
        dijkstra_path, dijkstra_distance, dijkstra_time = self.dijkstra(
            origin, dest)
        gbfs_path, gbfs_distance, gbfs_time = self.greedy_best_first_search(
            origin, dest)

        # compute time diffs/distance diffs
        time_diff = dijkstra_time - gbfs_time
        time_diff_percent = (time_diff / dijkstra_time) * \
            100 if dijkstra_time > 0 else 0

        distance_diff = gbfs_distance - dijkstra_distance
        distance_ratio = (
            distance_diff / dijkstra_distance) * 100 if dijkstra_distance > 0 else 0

        return {
            'dijkstra': {'path': dijkstra_path, 'distance': dijkstra_distance, 'time': dijkstra_time},
            'gbfs': {'path': gbfs_path, 'distance': gbfs_distance, 'time': gbfs_time},
            'comparison': {'time_diff': time_diff, 'time_diff_percent': time_diff_percent, 'distance_diff': distance_diff, 'distance_ratio': distance_ratio}
        }
