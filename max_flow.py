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