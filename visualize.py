import matplotlib.pyplot as plt


def visualize(net, filename='result.pdf', map=None):
    fig, ax = plt.subplots(figsize=(20, 20))
    ax.set_xlim([0, 1000])
    ax.set_ylim([0, 1000])
    for i in range(len(net.targets)):
        ax.add_patch(plt.Circle((net.targets[i].x, net.targets[i].y), net.r, color='r', alpha=0.2))
    plt.scatter([net.targets[i].x for i in range(len(net.targets))],
                [net.targets[i].y for i in range(len(net.targets))],
                marker='*', color='r', label='Target')
    plt.scatter([net.sList[i].x for i in range(len(net.sList))],
                [net.sList[i].y for i in range(len(net.sList))],
                marker='^', color='g', label='Sensor')
    plt.scatter([net.relay_nodes[i].x for i in range(len(net.relay_nodes))],
                [net.relay_nodes[i].y for i in range(len(net.relay_nodes))],
                marker='^', color='b', label='Relay')
    plt.plot([net.Base.x], [net.Base.y], 'yv', markersize=12, label='Base station')
    ax.set_aspect(1)
    if map is not None:
        ax.imshow(map, extent=[0, 1000, 0, 1000])
    plt.legend()
    plt.savefig(filename)
    plt.show()
