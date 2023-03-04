from solver import *
import json

OUTPUT_DIR = '.\\out.json'


class Generator:
    def __init__(self, num_target, q_max=5, r_s=40, r_c=80, r_cl=80, area=1000, rand_q=True, base=[0, 0]):
        self.base = Point(base[0], base[1])
        self.num_target = num_target
        self.q_max = q_max
        if rand_q:
            self.q = np.random.randint(1, q_max + 1, num_target).tolist()
        else:
            self.q = [q_max for i in range(num_target)]
        self.targets = []
        self.t_list = np.random.uniform(0, area, (num_target, 2)).tolist()
        for i in range(num_target):
            self.targets.append(Target(self.t_list[i][0], self.t_list[i][1], self.q[i], r_s))
        self.r_s = r_s
        self.r_c = r_c
        self.r_cl = r_cl
        self.area = area
        assert r_cl >= 2 * r_s

    def generate(self):
        net = Net(self.targets, self.r_s, self.r_c, self.r_cl, self.q, self.base)
        # Phase 1
        net.build_disk_set()
        net.cut_disk_set(k=2)
        net.place_sensor()
        # Phase 2
        net.createCluster()
        net.build_graph()
        net.insert_edge()
        net.place_relay_nodes_in_clusters()
        net.place_relay_nodes_between_centers()

        out_dict = dict()
        out_dict["targets"] = self.t_list
        out_dict["base"] = [self.base.x, self.base.y]

        out_dict["sensors"] = []
        for sensor in net.sList:
            out_dict["sensors"].append([sensor.x, sensor.y])

        out_dict["relays"] = []
        for relay in net.relay_nodes:
            out_dict["relays"].append([relay.x, relay.y])

        with open(OUTPUT_DIR, "w") as out_file:
            json.dump(out_dict, out_file)
