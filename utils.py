from max_flow import *
from shapely.geometry import Point
from ortools.linear_solver import pywraplp
import numpy as np
from math import pi, sin, cos, sqrt
import random
from entities import vector, Relay
import secrets


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