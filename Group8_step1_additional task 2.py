import math
import random
import matplotlib.pyplot as plt


show_animation = True


class AStarPlanner:
    def __init__(self, ox, oy, resolution, rr, fc_x, fc_y, tc_x, tc_y):
        self.resolution = resolution
        self.rr = rr
        self.min_x, self.min_y = 0, 0
        self.max_x, self.max_y = 0, 0
        self.obstacle_map = None
        self.x_width, self.y_width = 0, 0
        self.motion = self.get_motion_model()
        self.calc_obstacle_map(ox, oy)

        self.fc_x = fc_x
        self.fc_y = fc_y
        self.tc_x = tc_x
        self.tc_y = tc_y

        self.Delta_C1 = 0.2
        self.Delta_C2 = 1

        self.costPerGrid = 1

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x
            self.y = y
            self.cost = cost
            self.parent_index = parent_index

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.parent_index)

    def planning(self, sx, sy, gx, gy):
        start_node = self.Node(self.calc_xy_index(sx, self.min_x),
                               self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x),
                              self.calc_xy_index(gy, self.min_y), 0.0, -1)

        open_set, closed_set = {}, {}
        open_set[self.calc_grid_index(start_node)] = start_node

        while True:
            if len(open_set) == 0:
                print("Open set is empty..")
                return None, None, None
            c_id = min(
                open_set,
                key=lambda o: open_set[o].cost + self.calc_heuristic(self, goal_node,
                                                                    open_set[o]))
            current = open_set[c_id]

            if show_animation:
                plt.plot(self.calc_grid_position(current.x, self.min_x),
                         self.calc_grid_position(current.y, self.min_y), "xc")
                if len(closed_set.keys()) % 10 == 0:
                    plt.pause(0.001)

            if current.x == goal_node.x and current.y == goal_node.y:
                print("Total Trip time required -> ", current.cost)
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            del open_set[c_id]
            closed_set[c_id] = current

            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2] * self.costPerGrid, c_id)

                if self.calc_grid_position(node.x, self.min_x) in self.tc_x:
                    if self.calc_grid_position(node.y, self.min_y) in self.tc_y:
                        node.cost = node.cost + self.Delta_C1 * self.motion[i][2]

                if self.calc_grid_position(node.x, self.min_x) in self.fc_x:
                    if self.calc_grid_position(node.y, self.min_y) in self.fc_y:
                        node.cost = node.cost + self.Delta_C2 * self.motion[i][2]

                n_id = self.calc_grid_index(node)

                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node
                else:
                    if open_set[n_id].cost > node.cost:
                        open_set[n_id] = node

        rx, ry = self.calc_final_path(goal_node, closed_set)
        return rx, ry, goal_node.cost

    def calc_final_path(self, goal_node, closed_set):
        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [
            self.calc_grid_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index!= -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            parent_index = n.parent_index
        return rx, ry

    @staticmethod
    def calc_heuristic(self, n1, n2):
        w = 1.0
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        d = d * self.costPerGrid
        return d

    def calc_heuristic_maldis(n1, n2):
        w = 1.0
        dx = w * math.abs(n1.x - n2.x)
        dy = w * math.abs(n1.y - n2.y)
        return dx + dy

    def calc_grid_position(self, index, min_position):
        pos = index * self.resolution + min_position
        return pos

    def calc_xy_index(self, position, min_pos):
        return round((position - min_pos) / self.resolution)

    def calc_grid_index(self, node):
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.min_x)
        py = self.calc_grid_position(node.y, self.min_y)

        if px < self.min_x:
            return False
        elif py < self.min_y:
            return False
        elif px >= self.max_x:
            return False
        elif py >= self.max_y:
            return False

        if self.obstacle_map[node.x][node.y]:
            return False

        return True

    def calc_obstacle_map(self, ox, oy):
        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))

        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)

        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.x_width)]
        for ix in range(self.x_width):
            x = self.calc_grid_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_grid_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.rr:
                        self.obstacle_map[ix][iy] = True
                        break

    @staticmethod
    def get_motion_model():
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]
        return motion


def calculate_aircraft_cost(FC, T, FCR, CT, CC):
    C = (FC * T * FCR * T) + (CT * T) + CC
    return C


def generate_random_map():
    map_size = 70
    num_obstacles = 500
    fuel_area_size = 40

    ox, oy = [], []
    for _ in range(num_obstacles):
        x = random.randint(0, map_size - 1)
        y = random.randint(0, map_size - 1)
        ox.append(x)
        oy.append(y)

    # 生成随机起点和终点，并设置安全区
    while True:
        sx = random.randint(0, map_size - 1)
        sy = random.randint(0, map_size - 1)
        gx = random.randint(0, map_size - 1)
        gy = random.randint(0, map_size - 1)
        if math.hypot(sx - gx, sy - gy) >= 40:  # 修改距离限制为至少40
            break

    sx_safety_zone = set()
    for i in range(max(0, sx - 5), min(map_size, sx + 5 + 1)):
        for j in range(max(0, sy - 5), min(map_size, sy + 5 + 1)):
            sx_safety_zone.add((i, j))

    gx_safety_zone = set()
    for i in range(max(0, gx - 5), min(map_size, gx + 5 + 1)):
        for j in range(max(0, gy - 5), min(map_size, gy + 5 + 1)):
            gx_safety_zone.add((i, j))

    occupied_positions = {sx, gx}
    ox = [x for x in ox if (x, y) not in sx_safety_zone and (x, y) not in gx_safety_zone and (x, y) not in occupied_positions]
    oy = [y for y in oy if (x, y) not in sx_safety_zone and (x, y) not in gx_safety_zone and (x, y) not in occupied_positions]

    # 生成燃油高价区，确保不在起点和终点安全区内
    fc_x, fc_y = [], []
    while True:
        fc_start_x = random.randint(0, map_size - fuel_area_size)
        fc_start_y = random.randint(0, map_size - fuel_area_size)
        fc_x = []
        fc_y = []
        for i in range(fc_start_x, fc_start_x + fuel_area_size):
            for j in range(fc_start_y, fc_start_y + fuel_area_size):
                if (i, j) not in sx_safety_zone and (i, j) not in gx_safety_zone:
                    fc_x.append(i)
                    fc_y.append(j)
        if len(fc_x) > 0:
            break

    return ox, oy, sx, sy, gx, gy, fc_x, fc_y


def main():
    print(__file__ + " start the A star algorithm demo!!")

    ox, oy, sx, sy, gx, gy, fc_x, fc_y = generate_random_map()

    grid_size = 1
    robot_radius = 1

    a_star = AStarPlanner(ox, oy, grid_size, robot_radius, fc_x, fc_y, [], [])
    rx, ry, T = a_star.planning(sx, sy, gx, gy)

    if show_animation:
        plt.plot(fc_x, fc_y, "oy")
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "og")
        plt.plot(gx, gy, "xb")
        plt.grid(True)
        plt.axis("equal")
        plt.plot(rx, ry, "-r")
        plt.pause(0.001)
        plt.show()

    try:
        FCR_collect = [0.0, 0.0, 0.0]
        CC_collect = [0.0, 0.0, 0.0]
        PC_collect = [0, 0, 0]
        CT_collect = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        FC = float(0.8)
        # A321neo
        FCR_collect[0] = 54
        CC_collect[0] = float(1800)
        PC_collect[0] = 200
        CT_collect[0] = 10
        CT_collect[3] = 15
        CT_collect[6] = 20
        # A330
        FCR_collect[1] = 84
        CC_collect[1] = float(2000)
        PC_collect[1] = 300
        CT_collect[1] = 15
        CT_collect[4] = 21
        CT_collect[7] = 27
        # 350
        FCR_collect[2] = 90
        CC_collect[2] = 2500
        PC_collect[2] = 350
        CT_collect[2] = 20
        CT_collect[5] = 27
        CT_collect[8] = 34
        # NAME
        name = ["a", "b", "c"]
        name[0] = "A321neo"
        name[1] = "A330-900neo"
        name[2] = "A350-900"
        total_cost_collect = [0.0, 0.0, 0.0]
        # sne_1
        print("\nsenario 1")
        FC = 0.76
        for i in range(0, 3):
            flight = int(math.ceil(3000 / PC_collect[i]))
            if 12 < flight:
                print(name[i] + " is not suits in senario 1")
                total_cost_collect[i] = 10000000000000000000000
            else:
                total_cost_collect[i] = float(calculate_aircraft_cost(FC, T, FCR_collect[i], CT_collect[i + 3], CC_collect[i]))
                print("the toltal cost in sen_1 for " + name[i])
                print(total_cost_collect[i])
        c = min(total_cost_collect[0], total_cost_collect[1], total_cost_collect[2])
        final = total_cost_collect.index(c)
        print("the best one is " + name[final])
        # sne_2
        print("\nsenario 2")
        FC = 0.88
        for i in range(0, 3):
            flight = int(math.ceil(1250 / PC_collect[i]))
            if 15 < flight:
                print(name[i] + " is not suits in senario 2")
            else:
                total_cost_collect[i] = float(calculate_aircraft_cost(FC, T, FCR_collect[i], CT_collect[i + 6], CC_collect[i]))
                print("the toltal cost in sen_2 for " + name[i])
                print(total_cost_collect[i])
        c = min(total_cost_collect[0], total_cost_collect[1], total_cost_collect[2])
        final = total_cost_collect.index(c)
        print("the best one is " + name[final])
        # sne_3
        print("\nsenario 3")
        FC = 0.95
        for i in range(0, 3):
            flight = int(math.ceil(2500 / PC_collect[i]))
            if 25 < flight:
                print(name[i] + " is not suits in senario 3")
            else:
                total_cost_collect[i] = float(calculate_aircraft_cost(FC, T, FCR_collect[i], CT_collect[i], CC_collect[i]))
                print("the toltal cost in sen_3 for " + name[i])
                print(total_cost_collect[i])
        c = min(total_cost_collect[0], total_cost_collect[1], total_cost_collect[2])
        final = total_cost_collect.index(c)
        print("the best one is " + name[final])
    except ValueError:
        print("cost")


if __name__ == '__main__':
    main()