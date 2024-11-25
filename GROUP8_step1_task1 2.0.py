"""

A* grid planning

author: Atsushi Sakai(@Atsushi_twi)
        Nikos Kanargias (nkana@tee.gr)

See Wikipedia article (https://en.wikipedia.org/wiki/A*_search_algorithm)

This is the simple code for path planning class

"""



import math

import matplotlib.pyplot as plt

show_animation = True


class AStarPlanner:

    def __init__(self, ox, oy, resolution, rr, fc_x, fc_y, tc_x, tc_y):
        """
        Initialize grid map for a star planning

        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        resolution: grid resolution [m]
        rr: robot radius[m]
        """

        self.resolution = resolution # get resolution of the grid
        self.rr = rr # robot radis
        self.min_x, self.min_y = 0, 0
        self.max_x, self.max_y = 0, 0
        self.obstacle_map = None
        self.x_width, self.y_width = 0, 0
        self.motion = self.get_motion_model() # motion model for grid search expansion
        self.calc_obstacle_map(ox, oy)

        self.fc_x = fc_x
        self.fc_y = fc_y
        self.tc_x = tc_x
        self.tc_y = tc_y
        

        self.Delta_C1 = 0.2 # cost intensive area 1 modifier
        self.Delta_C2 = 1 # cost intensive area 2 modifier

        self.costPerGrid = 1 


    class Node: # definition of a sinle node
        def __init__(self, x, y, cost, parent_index):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.parent_index = parent_index

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.parent_index)

    def planning(self, sx, sy, gx, gy):
        """
        A star path search

        input:
            s_x: start x position [m]
            s_y: start y position [m]
            gx: goal x position [m]
            gy: goal y position [m]

        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """

        start_node = self.Node(self.calc_xy_index(sx, self.min_x), # calculate the index based on given position
                               self.calc_xy_index(sy, self.min_y), 0.0, -1) # set cost zero, set parent index -1
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x), # calculate the index based on given position
                              self.calc_xy_index(gy, self.min_y), 0.0, -1)

        open_set, closed_set = dict(), dict() # open_set: node not been tranversed yet. closed_set: node have been tranversed already
        open_set[self.calc_grid_index(start_node)] = start_node # node index is the grid index

        while 1:
            if len(open_set) == 0:
                print("Open set is empty..")
                break

            c_id = min(
                open_set,
                key=lambda o: open_set[o].cost + self.calc_heuristic(self, goal_node,
                                                                     open_set[
                                                                         o])) # g(n) and h(n): calculate the distance between the goal node and openset
            current = open_set[c_id]

            # show graph
            if show_animation:  # pragma: no cover
                plt.plot(self.calc_grid_position(current.x, self.min_x),
                         self.calc_grid_position(current.y, self.min_y), "xc")
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect('key_release_event',
                                             lambda event: [exit(
                                                 0) if event.key == 'escape' else None])
                if len(closed_set.keys()) % 10 == 0:
                    plt.pause(0.001)

            # reaching goal
            if current.x == goal_node.x and current.y == goal_node.y:
                print("Total Trip time required -> ",current.cost )
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            # Remove the item from the open set
            del open_set[c_id]

            # Add it to the closed set
            closed_set[c_id] = current

            # print(len(closed_set))

            # expand_grid search grid based on motion model
            for i, _ in enumerate(self.motion): # tranverse the motion matrix
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2] * self.costPerGrid, c_id)
                
                ## add more cost in cost intensive area 1
                if self.calc_grid_position(node.x, self.min_x) in self.tc_x:
                    if self.calc_grid_position(node.y, self.min_y) in self.tc_y:
                        # print("cost intensive area!!")
                        node.cost = node.cost + self.Delta_C1 * self.motion[i][2]
                
                # add more cost in cost intensive area 2
                if self.calc_grid_position(node.x, self.min_x) in self.fc_x:
                    if self.calc_grid_position(node.y, self.min_y) in self.fc_y:
                        # print("cost intensive area!!")
                        node.cost = node.cost + self.Delta_C2 * self.motion[i][2]
                    # print()
                
                n_id = self.calc_grid_index(node)

                # If the node is not safe, do nothing
                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node  # discovered a new node
                else:
                    if open_set[n_id].cost > node.cost:
                        # This path is the best until now. record it
                        open_set[n_id] = node

        rx, ry = self.calc_final_path(goal_node, closed_set)
        # print(len(closed_set))
        # print(len(open_set))

        return rx, ry ,goal_node.cost

    def calc_final_path(self, goal_node, closed_set):
        # generate final course
        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [
            self.calc_grid_position(goal_node.y, self.min_y)] # save the goal node as the first point
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            parent_index = n.parent_index

        return rx, ry

    @staticmethod
    def calc_heuristic(self, n1, n2):
        w = 1.0  # weight of heuristic
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        d = d * self.costPerGrid
        return d
    
    def calc_heuristic_maldis(n1, n2):
        w = 1.0  # weight of heuristic
        dx = w * math.abs(n1.x - n2.x)
        dy = w *math.abs(n1.y - n2.y)
        return dx + dy

    def calc_grid_position(self, index, min_position):
        """
        calc grid position

        :param index:
        :param min_position:
        :return:
        """
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

        # collision check
        if self.obstacle_map[node.x][node.y]:
            return False

        return True

    def calc_obstacle_map(self, ox, oy):

        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))
        print("min_x:", self.min_x)
        print("min_y:", self.min_y)
        print("max_x:", self.max_x)
        print("max_y:", self.max_y)

        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)
        print("x_width:", self.x_width)
        print("y_width:", self.y_width)

        # obstacle map generation
        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.x_width)] # allocate memory
        for ix in range(self.x_width):
            x = self.calc_grid_position(ix, self.min_x) # grid position calculation (x,y)
            for iy in range(self.y_width):
                y = self.calc_grid_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy): # Python’s zip() function creates an iterator that will aggregate elements from two or more iterables. 
                    d = math.hypot(iox - x, ioy - y) # The math. hypot() method finds the Euclidean norm
                    if d <= self.rr:
                        self.obstacle_map[ix][iy] = True # the griid is is occupied by the obstacle
                        break

    @staticmethod
    def get_motion_model(): # the cost of the surrounding 8 points
        # dx, dy, cost
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

def main():
    print(__file__ + " start the A star algorithm demo !!") # print simple notes
    # start and goal position
    sx = 0.0  # [m]
    sy = 0.0  # [m]
    gx = 50.0  # [m]
    gy = 30.0  # [m]
    grid_size = 1  # [m]
    robot_radius = 1.0  # [m]

    # set obstacle positions for group 8
    ox, oy = [], []
    for i in range(-10, 60): # draw the button border 
        ox.append(i)
        oy.append(-10.0)
    for i in range(-10, 60): # draw the right border
        ox.append(60.0)
        oy.append(i)
    for i in range(-10, 60): # draw the top border
        ox.append(i)
        oy.append(60.0)
    for i in range(-10, 60): # draw the left border
        ox.append(-10.0)
        oy.append(i)

    for i in range(20, 50): # draw the free border
        ox.append(40.0)
        oy.append(i)

    for i in range(0, 10):
        ox.append(i)
        oy.append(-2 * i + 10)

    for i in range(0,30):  #draw the 
        ox.append(i)
        oy.append(- i / 3+ 60)

    

    # set cost intesive area 1
    tc_x, tc_y = [], []
    for i in range(10, 30):
        for j in range(20, 40):
            tc_x.append(i)
            tc_y.append(j)
    
    # set cost intesive area 2
    fc_x, fc_y = [], []
    for i in range(30, 55):
        for j in range(0, 10):
            fc_x.append(i)
            fc_y.append(j)


    if show_animation:  # pragma: no cover
        plt.plot(ox, oy, ".k") # plot the obstacle
        plt.plot(sx, sy, "og") # plot the start position 
        plt.plot(gx, gy, "xb") # plot the end position
        
        plt.plot(fc_x, fc_y, "oy") # plot the cost intensive area 1
        plt.plot(tc_x, tc_y, "or") # plot the cost intensive area 2

        plt.grid(True) # plot the grid to the plot panel
        plt.axis("equal") # set the same resolution for x and y axis 

    a_star = AStarPlanner(ox, oy, grid_size, robot_radius, fc_x, fc_y, tc_x, tc_y)
    rx, ry, T= a_star.planning(sx, sy, gx, gy)

    if show_animation:  # pragma: no cover
        plt.plot(rx, ry, "-r") # show the route 
        plt.pause(0.001) # pause 0.001 seconds
        plt.show() # show the plot
    try:
        FCR_collect = [0.0,0.0,0.0]
        CC_collect = [0.0,0.0,0.0]
        PC_collect = [0,0,0]
        CT_collect = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0] #first three for min, second three for mid, ------
        FC = float(0.8)
        #A321neo
        FCR_collect[0] = 54
        CC_collect[0] = float(1800)
        PC_collect[0] = 200
        CT_collect[0] = 10
        CT_collect[3] = 15
        CT_collect[6] = 20
        #A330
        FCR_collect[1] = 84
        CC_collect[1] = float(2000)
        PC_collect[1] = 300
        CT_collect[1] = 15
        CT_collect[4] = 21
        CT_collect[7] = 27
        #350
        FCR_collect[2] = 90
        CC_collect[2] = 2500
        PC_collect[2] = 350
        CT_collect[2] = 20
        CT_collect[5] = 27
        CT_collect[8] = 34
        #NAME
        name = ["a","b","c"]
        name[0]= "A321neo"
        name[1]= "A330-900neo"
        name[2]= "A350-900"
        total_cost_collect=[0.0,0.0,0.0]
        #sne_1
        print("\nsenario 1")
        FC=0.76
        for i in range (0,3):
            flight = int(math.ceil(3000 / PC_collect[i]))
            if 12 < flight:
                print ( name[i] + " is not suits in senario 1")
                total_cost_collect[i] = 10000000000000000000000 #make it impassible
            else:
                total_cost_collect[i] = float(calculate_aircraft_cost(FC,T,FCR_collect[i],CT_collect[i+3],CC_collect[i]))
                print ("the total cost in sen_1 for " + name[i] )
                print (  total_cost_collect[i])
        c=min(total_cost_collect[0],total_cost_collect[1],total_cost_collect[2])
        final = total_cost_collect.index(c)
        print("the best one is " + name[final])
        #sne_2
        print("\nsenario 2")
        FC=0.88
        for i in range (0,3):
            flight = int(math.ceil(1250 / PC_collect[i]))
            if 15 < flight:
                print ( name[i] + " is not suits in senario 2")
            else:
                total_cost_collect[i] = float(calculate_aircraft_cost(FC,T,FCR_collect[i],CT_collect[i+6],CC_collect[i]))
                print ("the total cost in sen_2 for " + name[i] )
                print (  total_cost_collect[i] )
        c=min(total_cost_collect[0],total_cost_collect[1],total_cost_collect[2])
        final = total_cost_collect.index(c)
        print("the best one is " + name[final])
        #sne_3
        print("\nsenario 3")
        FC=0.95
        for i in range (0,3):
            flight = int(math.ceil(2500 / PC_collect[i]))
            if 25 < flight:
                print ( name[i] + " is not suits in senario 3")
            else:
                total_cost_collect[i] = float(calculate_aircraft_cost(FC,T,FCR_collect[i],CT_collect[i],CC_collect[i]))
                print ("the total cost in sen_3 for " + name[i] )
                print (  total_cost_collect[i] )
        c=min(total_cost_collect[0],total_cost_collect[1],total_cost_collect[2])
        final = total_cost_collect.index(c)
        print("the best one is " + name[i])
    except ValueError:
        print("cost")


if __name__ == '__main__':
    main()
