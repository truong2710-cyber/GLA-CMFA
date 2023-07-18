import copy
import random
import secrets
from shapely.geometry import Polygon
from entities import Sensor, Relay, Target
from utils import *


class Net:
    def __init__(self, targets, r, r_c, r_cl, q, Base):
        self.targets = targets
        self.n = len(targets)
        self.disk_set = []  # potential points
        self.r = r
        self.r_c = r_c
        self.r_cl = r_cl
        self.q = q
        self.Base = Base
        self.final_sensor_set = []  # optimized potential points
        self.min_s = 0
        self.LP_res = 0  # result: num of sensors
        self.regions = []
        self.distribution = []  # num of sensors in each region
        self.sList = []  # location of final sensor set (result of phase 1)
        self.clusters = []
        self.centers = []
        self.nodes_for_each_target = []
        self.next_vertex = []
        self.relay_nodes = []  # location of final relay node set (result of phase 2)

    def build_disk_set(self):
        n = len(self.targets)
        # find intersections and create adjacency matrix
        c = [[0 for i in range(n)] for j in range(n)]
        for i in range(n - 1):
            for j in range(i + 1, n):
                if self.targets[i].distance(self.targets[j]) <= 2 * (self.r) + 1e-9:
                    c[i][j] = 1
                    c[j][i] = 1
                    self.disk_set.extend(self.targets[i].intersect(self.targets[j]))
        self.disk_set = list(set(self.disk_set))
        # find the targets that each intersection point covers
        for s in self.disk_set:
            s.find_cover(self.targets)
        # if any target disk does not intersect, place one sensor inside that disk    
        for i in range(n):
            check = False
            for j in range(n):
                if c[i][j] == 1:
                    check = True
            if not check:
                x = self.targets[i].x + self.r - 1
                y = self.targets[i].y
                s = Sensor(x, y, self.r)
                s.cover.append(self.targets[i])
                self.disk_set.append(s)

    def cut_disk_set(self, k):
        # with each target, find k intersection points which cover the most targets
        # then append it to the final_sensor_set
        for t in self.targets:
            sub_disk_set = []
            for s in self.disk_set:
                if t in s.cover:
                    sub_disk_set.append(s)
            key = lambda s: len(s.cover)
            sub_disk_set.sort(key=key, reverse=True)
            sub_disk_set = sub_disk_set[:k]
            self.final_sensor_set.extend(sub_disk_set)
        self.final_sensor_set = set(self.final_sensor_set)
        
        # find the optimal regions from the final intersection set, refine the set and solve the LP 
        regions = []
        for s in self.final_sensor_set:
            regions.append([self.targets.index(t) for t in s.cover])
        self.regions = list(set(map(tuple, regions)))
        self.regions = delete(self.regions)
        self.distribution = solve(self.targets, self.regions, self.q)
        self.LP_res = sum(self.distribution)

    def place_sensor(self):
        for i in range(len(self.regions)):
            if len(self.regions[i]) == 1:
                num_s = self.distribution[i]
                for j in range(num_s):
                    self.sList.append(
                        Sensor(get_random_point_inside_circle(self.targets[self.regions[i][0]], self.r), self.r,
                               [self.targets[i] for i in self.regions[i]]))
                continue
            v = find_vertex(self.regions[i], self.targets, self.r)
            vertexes = [(s.x, s.y) for s in v]
            if len(vertexes) < 3:
                num_s = self.distribution[i]
                if len(vertexes) == 2:
                    for j in range(num_s):
                        self.sList.append(Sensor(get_random_point_between_2_points(vertexes[0], vertexes[1]), self.r,
                                                 [self.targets[i] for i in self.regions[i]]))
                elif len(vertexes) == 1:
                    for j in range(num_s):
                        self.sList.append(Sensor(get_random_point_between_2_points(vertexes[0], vertexes[0]), self.r,
                                                 [self.targets[i] for i in self.regions[i]]))
                continue
            poly = Polygon(vertexes)
            num_s = self.distribution[i]
            for j in range(num_s):
                self.sList.append(
                    Sensor(get_random_point_in_polygon(poly), self.r, [self.targets[i] for i in self.regions[i]]))

    def create_cluster(self):
        key = lambda t: t.q
        # sort to prioritize targets with large q (in case they have the same #neighbor)
        self.targets.sort(key=key,
                          reverse=True)  
        self.q = [t.q for t in self.targets]
        # intialize the node set in each target disk
        self.nodes_for_each_target = [[] for i in range(self.n)]
        for i in range(self.n):
            for j in range(self.LP_res):
                if self.targets[i].distance(self.sList[j]) <= self.r:
                    self.nodes_for_each_target[i].append(self.sList[j])
        clusters = []
        centers = []
        index = [i for i in range(self.n)]
        # loop to create cluster until there are no targets left
        while len(index) > 0:
            maxc = 0
            maxt = None
            for i in index:
                count = 0
                for j in index:
                    if distance(self.targets[i], self.targets[j]) <= self.r_cl:
                        count += 1
                if count > maxc:
                    maxc = count
                    maxt = i
            cluster = []
            centers.append(maxt)
            i = 0
            while i < len(index):
                if distance(self.targets[maxt], self.targets[index[i]]) <= self.r_cl:
                    cluster.append(index[i])
                    index.remove(index[i])
                    i = -1
                i += 1
            # place additional relay nodes (anchor nodes) inside the center disk if there are not enough nodes
            max_q = max([self.q[t] for t in cluster])
            if len(self.nodes_for_each_target[maxt]) < max_q:
                self.relay_nodes.extend([Relay(self.targets[maxt])] * (max_q - len(self.nodes_for_each_target[maxt])))
                self.nodes_for_each_target[maxt].extend(
                    [Relay(self.targets[maxt])] * (max_q - len(self.nodes_for_each_target[maxt])))
            else:
                self.nodes_for_each_target[maxt] = random.sample(self.nodes_for_each_target[maxt], max_q)
            clusters.append(cluster)
        self.clusters = clusters
        self.centers = centers

    def build_graph(self):
        # create the vertex list (the centers of clusters) with k elements
        tList_ext = self.targets.copy()
        tList_ext.append(self.Base)
        self.q_ext = [len(self.nodes_for_each_target[i]) for i in self.centers]  # wrong q_ext?
        max_q = max(self.q_ext)
        self.q_ext.append(max_q)
        self.centers_ext = self.centers.copy()
        self.centers_ext.append(self.n)
        k = len(self.centers_ext)

        # create the edge list and calculate the length of the edges
        self.adjacency_list = [[] for i in range(k)]
        edges = {}
        for i in range(k - 1):
            for j in range(i + 1, k):
                edges[(i, j)] = distance(tList_ext[self.centers_ext[i]], tList_ext[self.centers_ext[j]])
        sorted_tuples = sorted(edges.items(), key=lambda item: item[1])
        sorted_edges = {k: v for k, v in sorted_tuples}
        sorted_edges_copy = copy.deepcopy(sorted_edges)
        # insert edge to the final edge list until the condition about vertex degree is met.
        for edge in sorted_edges_copy.items():
            if deg(edge[0][0], self.adjacency_list) < self.q_ext[edge[0][0]] or deg(edge[0][1], self.adjacency_list) < \
                    self.q_ext[edge[0][1]]:
                self.adjacency_list[edge[0][0]].append(edge[0][1])
                self.adjacency_list[edge[0][1]].append(edge[0][0])
                sorted_edges.pop(edge[0])
        self.sorted_edges_left = sorted_edges
        self.next_vertex = [[] for i in range(len(self.clusters))]

    def insert_edge(self):
        k = len(self.centers)
        # loop through each vertex
        for i in range(k):
            # check the q_connect constraint of that vertex, add shortest edge left until the constraint is satisfied
            check_result = check_q_connect(i, self.adjacency_list, self.q_ext)
            if check_result[0] == False:
                while True:
                    insert_edge = next(iter(self.sorted_edges_left))
                    self.adjacency_list[insert_edge[0]].append(insert_edge[1])
                    self.adjacency_list[insert_edge[1]].append(insert_edge[0])
                    self.sorted_edges_left.pop(insert_edge)
                    check_result = check_q_connect(i, self.adjacency_list, self.q_ext)
                    if check_result[0]:
                        self.next_vertex[i] = random.sample(check_result[1], self.q_ext[i])
                        break
            else:
                self.next_vertex[i] = random.sample(check_result[1], self.q_ext[i])

    def place_relay_nodes_in_clusters(self):
        for i in range(len(self.clusters)):
            for t in self.clusters[i]:
                if t != self.centers[i]:
                    place_relay_nodes_between_sets(self.nodes_for_each_target[t],
                                                   self.nodes_for_each_target[self.centers[i]], self.q[t], self.r_c,
                                                   self.relay_nodes)

    def place_relay_nodes_between_centers(self):
        centers_ext = self.centers.copy()
        centers_ext.append(self.n)
        check_used_node = []
        for i in range(len(self.centers)):
            check = [0 for j in range(len(self.nodes_for_each_target[self.centers[i]]))]
            check_used_node.append(check)
        adj_mat = [[0 for i in range(len(centers_ext))] for j in range(len(centers_ext))]
        next_vertex_mat = [[0 for i in range(len(centers_ext))] for j in range(len(centers_ext))]
        for u in range(len(self.next_vertex)):
            for v in self.next_vertex[u]:
                next_vertex_mat[u][v] = 1
        for u in range(len(self.next_vertex)):
            for v in self.next_vertex[u]:
                if adj_mat[u][v] == 0 and v != len(self.centers):
                    node1 = get_unused_node(check_used_node[u], self.nodes_for_each_target, self.centers[u])
                    if next_vertex_mat[v][u] == 1:
                        node2 = get_unused_node(check_used_node[v], self.nodes_for_each_target, self.centers[v])
                    else:
                        node2 = secrets.choice(self.nodes_for_each_target[self.centers[v]])
                    if distance(node1, node2) > self.r_c:
                        place_relay_nodes_between_2_points(node1, node2, self.r_c, self.relay_nodes)
                    adj_mat[u][v] = 1
                    adj_mat[v][u] = 1
                elif v == len(self.centers):
                    node = get_unused_node(check_used_node[u], self.nodes_for_each_target, self.centers[u])
                    if distance(node, self.Base) > self.r_c:
                        place_relay_nodes_between_2_points(node, self.Base, self.r_c, self.relay_nodes)
                    adj_mat[u][v] = 1
                    adj_mat[v][u] = 1
        for u in range(len(self.adjacency_list)):
            for v in self.adjacency_list[u]:
                if u < v and adj_mat[u][v] == 0:
                    place_relay_nodes_between_2_targets(centers_ext[u], centers_ext[v], self.n, self.Base,
                                                        self.nodes_for_each_target, self.r_c, self.relay_nodes)
                    adj_mat[u][v] = 1
                    adj_mat[v][u] = 1
