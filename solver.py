import copy
import random
import secrets
import time
from math import sqrt, sin, cos, pi

import matplotlib.pyplot as plt
import numpy as np
from ortools.linear_solver import pywraplp
from shapely.geometry import Polygon, Point


def FindPath(V, f, s, t, c, n):
    # Find Augmentation Path

    p = {i: '0' for i in V}
    p[s] = s
    epsi = {i: 9999 for i in V}
    VT = [s]
    check = {i: 0 for i in V}
    check[s] = 1
    while len(VT) > 0:
        u = VT.pop(0)
        for v in V:
            if check[v] == 0:
                if c[u][v] > 0 and f[u][v] < c[u][v]:
                    p[v] = u
                    epsi[v] = min(epsi[u], c[u][v] - f[u][v])
                    VT.append(v)
                    check[v] = 1

                if c[v][u] > 0 and f[v][u] > 0:
                    p[v] = '-' + u
                    epsi[v] = min(epsi[u], f[v][u])
                    VT.append(v)
                    check[v] = 1
    if check[t] == 1:
        return (p, epsi)
    return [False]


def IncFlow(V, f, s, t, c, n):
    # Increase Flow using Augmentation Path
    p, epsi = FindPath(V, f, s, t, c, n)
    u = p[t]
    v = t
    inc = epsi[t]
    while v != s:
        if u.startswith("-") == False:
            f[u][v] += inc
        else:
            u = u.replace('-', '')
            f[v][u] -= inc
        v = u
        u = p[v]


def MaxFlow(s, t, c, n, V):
    """
    input:
      s: source (string)
      t: target (string)
      c: adjacency matrix
      n: number of vertexes
      V: vertex list (string)
    output:
      max flow from s to t
    """

    f = {i: {j: 0 for j in V} for i in V}  # flow
    stop = False
    while stop == False:
        if len(FindPath(V, f, s, t, c, n)) > 1:
            IncFlow(V, f, s, t, c, n)
        else:
            stop = True
    max_flow = 0
    for u in V:
        max_flow += f[s][u]
    return max_flow, f


def check_q_connect(i, adjacency_list, q_ext):
    k = len(adjacency_list)
    s = str(i)
    t = str(k - 1)
    V = [s, t]
    for u in range(len(adjacency_list) - 1):
        if str(u) != s:
            V.extend([str(u) + "'", str(u) + "''"])
    N = len(V)
    c = {u: {v: 0 for v in V} for u in V}
    for u in range(k):
        for v in adjacency_list[u]:
            if str(u) != s and str(u) != t and str(v) != s and str(v) != t:
                c[str(u) + "'"][str(u) + "''"] = 1
                c[str(v) + "'"][str(v) + "''"] = 1
                c[str(u) + "''"][str(v) + "'"] = 1
                c[str(v) + "''"][str(u) + "'"] = 1
            elif str(u) == s and str(v) != t:
                c[str(v) + "'"][str(v) + "''"] = 1
                c[str(u)][str(v) + "'"] = 1
            elif str(u) == s and str(v) == t:
                c[str(u)][str(v)] = 1
            elif str(u) == t and str(v) != s:
                c[str(v) + "'"][str(v) + "''"] = 1
                c[str(v) + "''"][str(u)] = 1
            elif str(u) == t and str(v) == s:
                c[str(v)][str(u)] = 1
    max_flow, f = MaxFlow(s, t, c, N, V)
    next_vertex = []
    for u in V:
        if f[s][u] == 1:
            next_vertex.append(int(u.strip("'")))
    return [max_flow >= q_ext[i], next_vertex]


def distance(a, b):
    return sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2)


def get_random_point_in_polygon(poly):
    minx, miny, maxx, maxy = poly.bounds
    while True:
        p = Point(random.uniform(minx, maxx), random.uniform(miny, maxy))
        if poly.contains(p):
            return p


def get_random_point_between_2_points(p1, p2):
    u = np.random.rand()
    return Point(u * p1[0] + (1 - u) * p2[0], u * p1[1] + (1 - u) * p2[1])


def get_random_point_inside_circle(t, r):
    u = np.random.rand()
    r_rand = r * u
    phi = 2 * pi * u
    x = t.x + r_rand * sin(phi)
    y = t.y + r_rand * cos(phi)
    return Point(x, y)


def get_unused_node(check, nodes_for_each_target, i):
    for j in range(len(nodes_for_each_target[i])):
        if (check[j] == 0):
            check[j] = 1
            return nodes_for_each_target[i][j]
    print("All used")


def isSubset(x, region):
    for i in x:
        if i not in region:
            return False
    return True


def delete(regions):
    erase = []
    for x in regions:
        for region in regions:
            if x != region and isSubset(x, region):
                erase.append(x)
    for x in erase:
        if x in regions:
            regions.remove(x)
    return regions


def solve(tList, regions, q):
    solver = pywraplp.Solver.CreateSolver('SCIP')
    x = [[]] * len(regions)
    for i in range(len(regions)):
        x[i] = solver.IntVar(0, solver.infinity(), ' ')
    for j in range(len(tList)):
        solver.Add(solver.Sum([x[i] for i in range(len(regions)) if j in regions[i]]) >= q[j])
    M = solver.Sum(x)
    opjective = solver.Minimize(M)
    solver.Solve()
    # for i in range(len(regions)):
    #    print(int(x[i].solution_value()),end=' ')
    return [int(x[i].solution_value()) for i in range(len(regions))]


def check_cover_all(point, region, tList, r):
    for i in region:
        if tList[i].distance(point) > r + 0.0001:
            return False
    return True


def deg(i, adjacency_mat):
    return len(adjacency_mat[i])


def find_vertex(region, tList, r):
    vertexes = []
    for i in region:
        t = tList[i]
        intersect_points = []
        for t_ in tList:
            if 0.0001 < t.distance(t_) <= 2 * r + 0.00001:
                intersect_points.extend(t.intersect(t_))
        for point in intersect_points:
            if check_cover_all(point, region, tList, r) == True:
                vertexes.append(point)
    return list(set(vertexes))


def place_relay_nodes_between_2_points(s1, s2, Rc, relay_nodes):
    if distance(s1, s2) == 0:
        return
    k = int(distance(s1, s2) / Rc)
    unit_vector = vector((s2.x - s1.x) / distance(s1, s2), (s2.y - s1.y) / distance(s1, s2))
    for i in range(1, k + 1):
        relay_nodes.append(Relay(s1.x + unit_vector.x * i * Rc, s1.y + unit_vector.y * i * Rc))


def nearest_sensor_in_set(s, sSet):
    min = float('inf')
    nearest = None
    for s1 in sSet:
        if distance(s, s1) < min:
            nearest = s1
            min = distance(s, s1)
    return nearest


def place_relay_nodes_between_sets(sSet1, sSet2, num, Rc,
                                   relay_nodes):  # them tham so q: chi lay q sList_s trong sset1 do co the dat thua (>q)
    sSet2_copy = sSet2.copy()
    sSet1_num = random.sample(sSet1, num)  # lay ngau nhien
    for s in sSet1_num:
        s1 = nearest_sensor_in_set(s, sSet2_copy)
        if distance(s, s1) > Rc:
            place_relay_nodes_between_2_points(s, s1, Rc, relay_nodes)
        sSet2_copy.remove(s1)


def place_relay_nodes_between_2_targets(i, j, n, Base, nodes_for_each_target, Rc, relay_nodes):
    # place relay sList_s between tList[i] and tList[j] (might include Base)
    if i != n and j != n:
        s1 = secrets.choice(nodes_for_each_target[i])
        s2 = secrets.choice(nodes_for_each_target[j])
        # relay_nodes.append(Relay(s1.x,s1.y))
        # relay_nodes.append(Relay(s2.x,s2.y))
        if distance(s1, s2) > Rc:
            place_relay_nodes_between_2_points(s1, s2, Rc, relay_nodes)
    elif i == n:
        s1 = secrets.choice(nodes_for_each_target[j])
        # relay_nodes.append(Relay(s1.x,s1.y))
        if distance(s1, Base) > Rc:
            place_relay_nodes_between_2_points(s1, Base, Rc, relay_nodes)
    elif j == n:
        s1 = secrets.choice(nodes_for_each_target[i])
        # relay_nodes.append(Relay(s1.x,s1.y))
        if distance(s1, Base) > Rc:
            place_relay_nodes_between_2_points(s1, Base, Rc, relay_nodes)


class vector:
    def __init__(self, x, y):
        self.x = x
        self.y = y


class Sensor:
    def __init__(self, *args):
        if (not isinstance(args[0], Point)):
            self.x = args[0]
            self.y = args[1]
            self.r = args[2]
            self.cover = []
        else:
            self.x = args[0].x
            self.y = args[0].y
            self.r = args[1]
            self.cover = args[2]

    def __hash__(self):
        return hash((self.x, self.y))

    def __eq__(self, another):
        if not isinstance(another, type(self)):
            return NotImplemented
        return self.x == another.x and self.y == another.y

    def find_cover(self, targets):
        for t in targets:
            if t.distance(self) <= self.r + 0.0001:
                self.cover.append(t)


class Relay:
    def __init__(self, *args):
        if len(args) == 2:
            self.x = args[0]
            self.y = args[1]
        else:
            self.x = args[0].x
            self.y = args[0].y


class Target:
    def __init__(self, x, y, q, r):
        self.x = x
        self.y = y
        self.q = q
        self.r = r
        self.o_q = q

    def distance(self, another):
        return sqrt((self.x - another.x) ** 2 + (self.y - another.y) ** 2)

    def intersect(self, another):
        if abs(self.distance(another) - 2 * self.r) < 0.0000001:
            return [Sensor((self.x + another.x) / 2, (self.y + another.y) / 2, self.r)]
        if self.distance(another) < 2 * self.r:
            m = Point((self.x + another.x) / 2, (self.y + another.y) / 2)
            d = sqrt(self.r ** 2 - ((self.x - another.x) ** 2 + (self.y - another.y) ** 2) / 4)
            n1 = d / sqrt((self.x - another.x) ** 2 + (self.y - another.y) ** 2)
            n2 = -n1
            i1 = Sensor((m.x + (self.y - another.y) * n1), (m.y + (another.x - self.x) * n1), self.r)
            i2 = Sensor((m.x + (self.y - another.y) * n2), (m.y + (another.x - self.x) * n2), self.r)
            return [i1, i2]


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
        c = [[0 for i in range(n)] for j in range(n)]
        for i in range(n - 1):
            for j in range(i + 1, n):
                if self.targets[i].distance(self.targets[j]) <= 2 * (self.r) + 0.000000001:
                    c[i][j] = 1
                    c[j][i] = 1
                    self.disk_set.extend(self.targets[i].intersect(self.targets[j]))
        self.disk_set = list(set(self.disk_set))
        for s in self.disk_set:
            s.find_cover(self.targets)
        for i in range(n):
            check = False
            for j in range(n):
                if c[i][j] == 1:
                    check = True
            if check == False:
                x = self.targets[i].x + self.r - 1
                y = self.targets[i].y
                s = Sensor(x, y, self.r)
                s.cover.append(self.targets[i])
                self.disk_set.append(s)

    def cut_disk_set(self, k):
        #        key=lambda t:t.q
        #        self.targets.sort(key=key)
        n = len(self.targets)

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

        # self.final_sensor_set=self.disk_set
        regions = []
        for s in self.final_sensor_set:
            regions.append([self.targets.index(t) for t in s.cover])
        self.regions = list(set(map(tuple, regions)))
        self.regions = delete(self.regions)
        #       self.q=[t.q for t in self.targets]   #target da sort nen phai chinh lai q
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

    def createCluster(self):
        key = lambda t: t.q
        self.targets.sort(key=key,
                          reverse=True)  # sap xep de cluster uu tien target co q lon trc (trong truong hop co cung #neighbor)
        self.q = [t.q for t in self.targets]
        self.nodes_for_each_target = [[] for i in range(self.n)]
        for i in range(self.n):
            for j in range(self.LP_res):
                if self.targets[i].distance(self.sList[j]) <= self.r:
                    self.nodes_for_each_target[i].append(self.sList[j])
        clusters = []
        centers = []
        index = [i for i in range(self.n)]
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
        tList_ext = self.targets.copy()
        tList_ext.append(self.Base)
        # q_ext=[len(sl) for sl in nodes_for_each_target]
        self.q_ext = [len(self.nodes_for_each_target[i]) for i in self.centers]  # wrong q_ext?
        max_q = max(self.q_ext)
        self.q_ext.append(max_q)
        self.centers_ext = self.centers.copy()
        self.centers_ext.append(self.n)
        k = len(self.centers_ext)
        # c=[[0 for i in range(k)] for j in range(k)]
        self.adjacency_list = [[] for i in range(k)]
        edges = {}
        for i in range(k - 1):
            for j in range(i + 1, k):
                edges[(i, j)] = distance(tList_ext[self.centers_ext[i]], tList_ext[self.centers_ext[j]])
        sorted_tuples = sorted(edges.items(), key=lambda item: item[1])
        sorted_edges = {k: v for k, v in sorted_tuples}
        sorted_edges_copy = copy.deepcopy(sorted_edges)
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
        for i in range(k):
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
        # max_q=max([q[i] for i in centers])
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
