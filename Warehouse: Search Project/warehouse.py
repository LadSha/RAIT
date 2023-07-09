######################################################################
# This file copyright the Georgia Institute of Technology
#
# Permission is given to students to use or modify this file (only)
# to work on their assignments.
#
# You may NOT publish this file or make it available to others not in
# the course.
#
######################################################################


import math

# If you see different scores locally and on Gradescope this may be an indication
# that you are uploading a different file than the one you are executing locally.
# If this local ID doesn't match the ID on Gradescope then you uploaded a different file.
OUTPUT_UNIQUE_FILE_ID = False
if OUTPUT_UNIQUE_FILE_ID:
    import hashlib, pathlib

    file_hash = hashlib.md5(pathlib.Path(__file__).read_bytes()).hexdigest()
    print(f'Unique file ID: {file_hash}')

def heuristic(grid_coor, goal_coor):
    return abs(goal_coor[0] - grid_coor[0]) + abs(goal_coor[1] - grid_coor[1])

class DeliveryPlanner_PartA:
    """
    Required methods in this class are:
    
      plan_delivery(self, debug = False) which is stubbed out below.  
        You may not change the method signature as it will be called directly 
        by the autograder but you may modify the internals as needed.
    
      __init__: which is required to initialize the class.  Starter code is 
        provided that initializes class variables based on the definitions in
        testing_suite_partA.py.  You may choose to use this starter code
        or modify and replace it based on your own solution.
        You should't change the signature, however.
    
    The following methods are starter code you may use for part A.  
    However, they are not required and can be replaced with your
    own methods.

      _search(self, debug=False): Where the bulk of the A* search algorithm
          could reside.  It should find an optimal path from the robot
          location to a goal.
          Hint:  you may want to structure this based
          on whether looking for a box or delivering a box.
  
    """

    ## Definitions taken from testing_suite_partA.py
    ORTHOGONAL_MOVE_COST = 2
    DIAGONAL_MOVE_COST = 3
    BOX_LIFT_COST = 4
    BOX_DOWN_COST = 2
    ILLEGAL_MOVE_PENALTY = 100

    def __init__(self, warehouse, robot_position, todo, box_locations):

        self.todo = todo
        self.boxes_delivered = []
        self.total_cost = 0
        self.warehouse_viewer = warehouse
        self.box_locations = box_locations
        self.dropzone = self.robot_position = robot_position

        self.delta = [[-1, 0],  # north
                      [0, -1],  # west
                      [1, 0],  # south
                      [0, 1],  # east
                      [-1, -1],  # northwest (diag)
                      [-1, 1],  # northeast (diag)
                      [1, 1],  # southeast (diag)
                      [1, -1]]  # southwest (diag)

        self.delta_directions = ["n", "w", "s", "e", "nw", "ne", "se", "sw"]

        # Can use this for a visual debug
        self.delta_name = ['^', '<', 'v', '>', '\\', '/', '[', ']']
        # You may choose to use arrows instead
        # self.delta_name = ['🡑', '🡐', '🡓', '🡒',  '🡔', '🡕', '🡖', '🡗']

        # Costs for each move
        self.delta_cost = [self.ORTHOGONAL_MOVE_COST,
                           self.ORTHOGONAL_MOVE_COST,
                           self.ORTHOGONAL_MOVE_COST,
                           self.ORTHOGONAL_MOVE_COST,
                           self.DIAGONAL_MOVE_COST,
                           self.DIAGONAL_MOVE_COST,
                           self.DIAGONAL_MOVE_COST,
                           self.DIAGONAL_MOVE_COST]


    def _search(self, end, starting, obj, box ,debug=False):
        """
        This method should be based on lesson modules for A*, see Search, Section 12-14.
        The bulk of the search logic should reside here, should you choose to use this starter code.
        Please condition any printout on the debug flag provided in the argument.  
        You may change this function signature (i.e. add arguments) as 
        necessary, except for the debug argument which must remain with a default of False
        """

        # get a shortcut variable for the warehouse (note this is just a view no copying)
        
        closed = set()
        

        x = starting[0]
        y = starting[1]

        g = 0
        h = heuristic(starting, end)
        f = g + h

        open = [[f, g, h, x, y]]
        found = False
        resign = False
        count = 0
        closed = set()
        action = {}
        expand = []
        while not found and not resign:
            if len(open) == 0:
                resign = True
            else:
                open.sort(reverse=True)
                next = open.pop()
                x = next[3]
                y = next[4]
                g = next[1]
            
                expand.append((x, y, count))
                count+=1
                if x==end[0] and y == end[1]:
                    
                    self.warehouse_viewer[x][y] = "."
                    found = True
                    
                for a in range(len(self.delta)):
                    new_x = x + self.delta[a][0]
                    new_y = y + self.delta[a][1]
                    cost = self.delta_cost[a]
                    if new_x==end[0] and new_y==end[1]:
                        self.warehouse_viewer[new_x][new_y] = "."
                        if obj=="pick":
                            cost = DeliveryPlanner_PartA.BOX_LIFT_COST
                        else:
                            cost = DeliveryPlanner_PartA.BOX_DOWN_COST
                    if self.warehouse_viewer[new_x][new_y]=="." and (new_x, new_y) not in closed:
                        g2 = g + cost
                        h2 = heuristic([new_x, new_y], end)
                        f2 = g2 + h2
                        open.append([f2, g2,h2,new_x,new_y])
                        closed.add((new_x, new_y))
                        action[(new_x, new_y)] = a
        
        moves = []
        x = end[0]
        y = end[1]
        a = action[(x, y)]
        current_loc = (x - self.delta[a][0], y - self.delta[a][1])

        while x!=starting[0] or y!=starting[1]:
            a = action[(x, y)]
            x2 = x - self.delta[a][0]
            y2 = y - self.delta[a][1]
            moves.append("move " + self.delta_directions[a])
            x = x2
            y = y2
        moves.reverse()
        moves.pop()
        if obj == "pick":
            moves.append("lift " + str(box))
        else:
            moves.append("down " + self.delta_directions[self.delta.index([end[0]-current_loc[0], end[1]-current_loc[1]])])



                 
            
                    















        # moves = []

        # closed = {} #[[0 for row in range(len(grid[0]))] for col in range(len(grid))]

        # closed[(starting[0],starting[1])] = 1

        # expand = {}# [[-1 for row in range(len(grid[0]))] for col in range(len(grid))]
        # action = {}#[[-1 for row in range(len(grid[0]))] for col in range(len(grid))]
        # # policy = [[-1 for row in range(len(grid[0]))] for col in range(len(grid))]

        # x = starting[0]
        # y = starting[1]
        # act = 0

        # g = 0
        # h = heuristic(starting, end)
        # f = g + h
        # open = [[f, g, h, y, x, act, act]]
        # found = False
        # resign = False
        # count = 0

        # while not found and not resign:
        #     if len(open) == 0:
        #         resign = True

        #     else:
             
        #         open.sort()
        #         open.reverse()

        #         next = open.pop()

        #         x = next[4]
        #         y = next[3]
        #         g = next[1]
        #         act_made = next[5]
                
        #         current_loc = (x, y)

        #         expand[(x,y)] = count

        #         if act_made != 0:
        #             moves.append(act_made)
        #             action[(x,y)] = next[6]
        #         count += 1

        #         if -1 <= x - end[0] <= 1 and -1 <= y - end[1] <= 1 and [end[0]-x,end[1]-y]!=[0,0]:
        #             found = True

        #             if obj == 'pick':
        #                 obj_action = 'lift ' + str(box)
        #                 grid[end[0]][end[1]] = '.'
                        

        #             elif obj == 'drop':
        #                 dir_x = end[0] - x
        #                 dir_y = end[1] - y
        #                 dir = [dir_x, dir_y]

        #                 if dir==[0,0]:
        #                     #break
                            
        #                     obj_action='need to go to diff cell'

        #                 else:
        #                     dir_facing = self.delta.index(dir)
        #                     obj_action = 'down ' + self.delta_directions[dir_facing]

        #             moves.append(obj_action)

               
        #         if abs(x - end[0])<=1 and abs(y - end[1])<=1 and [x - end[0],y - end[1]]!=[0,0]:
        #             found = True

        #         else:
        #             for i in range(len(self.delta)):
                       
        #                 x2 = x + self.delta[i][0]
        #                 y2 = y + self.delta[i][1]
        #                 if x2 == end[0] and y2==end[0]:
        #                     if obj == "pick":
        #                         cost2 = DeliveryPlanner_PartA.BOX_LIFT_COST
        #                     else:
        #                         cost2 = DeliveryPlanner_PartA.BOX_DOWN_COST
        #                 else:
        #                     cost2 = self.delta_cost[i]

        #                 if (x2,y2) not in closed and grid[x2][y2] == '.':
    
        #                     g2 = g + cost2
        #                     h2 = heuristic([x2, y2], end)
        #                     f2 = g2 + h2
        #                     act_format = "move " + self.delta_directions[i]
        #                     open.append([f2, h2, g2, y2, x2, act_format, i])
        #                     closed[(x2,y2)] = 1
            

        # y,x=current_loc

        # move_plan=[]
        # if obj == 'pick':
        #     obj_action = 'lift ' + str(box)
        #     grid[end[0]][end[1]] = '.'

        # elif obj == 'drop':
        #     dir_y = end[0] - y
        #     dir_x = end[1] - x
        #     dir = [dir_y, dir_x]
        #     if abs(dir_y)<=1 and abs(dir_x)<=1:
        #         dir_facing = self.delta.index(dir)
        #         obj_action = 'down ' + self.delta_directions[dir_facing]
        #     else:
        #         dir_facing='drop obj unable to compute y x' +str(dir_y) + str(dir_x)
        #         obj_action = dir_facing

        # move_plan.append(obj_action)


        # while abs(y - starting[0]) != 0 or  abs(x - starting[1]) !=0:
        #     move_made='move ' + self.delta_directions[action[(y,x)]]
        #     move_plan.append(move_made)
        #     x2=x-self.delta[action[(y,x)]][1]
        #     y2=y-self.delta[action[(y,x)]][0]
        #     # policy[y2][x2]=self.delta_directions[action[y][x]]
        #     x=x2
        #     y=y2

        # move_plan.reverse()
        
        return moves, current_loc

        

    def plan_delivery(self, debug=False):
        """
        plan_delivery() is required and will be called by the autograder directly.  
        You may not change the function signature for it.
        Add logic here to find the moves.  You may use the starter code provided
        in any way you choose, but please condition any printouts on the debug flag
        """

        # You may wish to break the task into one-way paths, like this:
        #
        #    moves_to_1   = self._search( ..., debug=debug )
        #    moves_from_1 = self._search( ..., debug=debug )
        #    moves_to_2   = self._search( ..., debug=debug )
        #    moves_from_2 = self._search( ..., debug=debug )
        #    moves        = moves_to_1 + moves_from_1 + moves_to_2 + moves_from_2
        #
        # If you use _search(), you may need to modify it to take some
        # additional arguments for starting location, goal location, and
        # whether to pick up or deliver a box.


        path = []
        current_loc = self.robot_position

        for item in self.todo:

            path_addition, current_loc = self._search(self.box_locations.get(item), current_loc, 'pick', item)
            path += path_addition
            

            path_addition, current_loc = self._search(self.dropzone, current_loc, 'drop', item)

            path += path_addition

        print(path)
        return path


class DeliveryPlanner_PartB:
    """
    Required methods in this class are:

        plan_delivery(self, debug = False) which is stubbed out below.
        You may not change the method signature as it will be called directly
        by the autograder but you may modify the internals as needed.

        __init__: required to initialize the class.  Starter code is
        provided that initializes class variables based on the definitions in
        testing_suite_partB.py.  You may choose to use this starter code
        or modify and replace it based on your own solution

    The following methods are starter code you may use for part B.
    However, they are not required and can be replaced with your
    own methods.

        _set_initial_state_from(self, warehouse): creates structures based on
            the warehouse and todo definitions and initializes the robot
            location in the warehouse

        _find_policy(self, goal, pickup_box=True, debug=False): Where the bulk 
            of the dynamic programming (DP) search algorithm could reside.  
            It should find an optimal path from the robot location to a goal.
            Hint:  you may want to structure this based
            on whether looking for a box or delivering a box.

    """

    # Definitions taken from testing_suite_partA.py
    ORTHOGONAL_MOVE_COST = 2
    DIAGONAL_MOVE_COST = 3
    BOX_LIFT_COST = 4
    BOX_DOWN_COST = 2
    ILLEGAL_MOVE_PENALTY = 100

    def __init__(self, warehouse, warehouse_cost, todo):

        self.todo = todo
        self.boxes_delivered = []
        self.total_cost = 0
        self._set_initial_state_from(warehouse)
        self.warehouse_cost = warehouse_cost

        self.delta = [[-1, 0],  # go up
                      [0, -1],  # go left
                      [1, 0],  # go down
                      [0, 1],  # go right
                      [-1, -1],  # up left (diag)
                      [-1, 1],  # up right (diag)
                      [1, 1],  # dn right (diag)
                      [1, -1]]  # dn left (diag)

        self.delta_directions = ["n", "w", "s", "e", "nw", "ne", "se", "sw"]

        # Use this for a visual debug
        self.delta_name = ['^', '<', 'v', '>', '\\', '/', '[', ']']
        # You may choose to use arrows instead
        # self.delta_name = ['🡑', '🡐', '🡓', '🡒',  '🡔', '🡕', '🡖', '🡗']

        # Costs for each move
        self.delta_cost = [self.ORTHOGONAL_MOVE_COST,
                           self.ORTHOGONAL_MOVE_COST,
                           self.ORTHOGONAL_MOVE_COST,
                           self.ORTHOGONAL_MOVE_COST,
                           self.DIAGONAL_MOVE_COST,
                           self.DIAGONAL_MOVE_COST,
                           self.DIAGONAL_MOVE_COST,
                           self.DIAGONAL_MOVE_COST]

    # state parsing and initialization function
    def _set_initial_state_from(self, warehouse):
        """Set initial state.

        Args:
            warehouse(list(list)): the warehouse map.
        """
        rows = len(warehouse)
        cols = len(warehouse[0])

        self.warehouse_state = [[None for j in range(cols)] for i in range(rows)]
        self.dropzone = None
        self.boxes = dict()

        for i in range(rows):
            for j in range(cols):
                this_square = warehouse[i][j]

                if this_square == '.':
                    self.warehouse_state[i][j] = '.'

                elif this_square == '#':
                    self.warehouse_state[i][j] = '#'

                elif this_square == '@':
                    self.warehouse_state[i][j] = '@'
                    self.dropzone = (i, j)

                else:  # a box
                    box_id = this_square
                    self.warehouse_state[i][j] = box_id
                    self.boxes[box_id] = (i, j)

    def _find_policy(self, goal, pickup_box=False, debug=False):
        """
        This method should be based on lesson modules for Dynamic Programming,
        see Search, Section 15-19 and Problem Set 4, Question 5.  The bulk of
        the logic for finding the policy should reside here should you choose to
        use this starter code.  Please condition any printout on the debug flag
        provided in the argument. You may change this function signature
        (i.e. add arguments) as necessary, except for the debug argument which
        must remain with a default of False
        """

        ##############################################################################
        # insert code in this method if using the starter code we've provided
        ##############################################################################


        # get a shortcut variable for the warehouse (note this is just a view it does not make a copy)
        grid = self.warehouse_state
        grid_costs = self.warehouse_cost
        for col in range(len(grid[0])):
            for row in range(len(grid)):
                if self.warehouse_cost[row][col]=="w":
                    grid_costs[row][col] = float("inf")
        

        # You will need to fill in the algorithm here to find the policy
        # The following are what your algorithm should return for test case 1
        collision_cost = 100
        if pickup_box:
            # To box policy
            pass
            value = [[float("inf") for col in range(len(grid[0]))] for row in range(len(grid))]
            policy = [[" " for col in range(len(grid[0]))] for row in range(len(grid))]

            change = True
            
            while change:
                change = False
                for x in range(len(grid)):
                    for y in range(len(grid[0])):
                        if goal[0] == x and goal[1] == y:
                            if value[x][y] != 0:
                                value[x][y] = 0
                                policy[x][y] = "B"
                                change = True

                        elif grid[x][y] == "." or grid[x][y] == "@":
                            for a in range(len(self.delta)):
                                new_x = x + self.delta[a][0]
                                new_y = y + self.delta[a][1]
                                if new_y>=len(grid[0]) or new_y<0 or new_x>=len(grid) or new_x<0 or grid[new_x][new_y]=="#":
                                    floor_cost = collision_cost
                                    v2 = self.delta_cost[a] + floor_cost + value[x][y]
                                elif new_x==goal[0] and new_y == goal[1]:
                                    v2 = DeliveryPlanner_PartB.BOX_LIFT_COST + value[new_x][new_y]
                                
                                else:
                                    floor_cost = grid_costs[new_x][new_y]
                                    v2 = self.delta_cost[a] + floor_cost + value[new_x][new_y]

                                # print (v2)

                                if v2 < value[x][y]:
                                    change = True
                                    value[x][y] = v2
                                    policy[x][y] = "move " + self.delta_directions[a]   
                                if new_y==goal[1] and new_x==goal[0]:
                                    policy[x][y] = "lift " + grid[new_x][new_y]
                        else:
                            policy[x][y] = "-1"
                
                
       
        else:
            # Deliver policy
            value = [[float("inf") for col in range(len(grid[0]))] for row in range(len(grid))]
            policy = [[" " for col in range(len(grid[0]))] for row in range(len(grid))]

            change = True
            
            while change:
                change = False
                
                for x in range(len(grid)):
                    for y in range(len(grid[0])):
                        if goal[0] == x and goal[1] == y:
                            if value[x][y] != 0:
                                value[x][y] = 0
                                
                                change = True
                        target_neighbors_min_value = float("inf")
                        if grid[x][y] != "#":
                            for a in range(len(self.delta)):
                                new_x = x + self.delta[a][0]
                                new_y = y + self.delta[a][1]
                                if new_y>=len(grid[0]) or new_y<0 or new_x>=len(grid) or new_x<0 or grid[new_x][new_y]=="#":
                                    floor_cost = collision_cost
                                    v2 = self.delta_cost[a] + floor_cost + value[x][y]
                                elif new_x==goal[0] and new_y == goal[1]:
                                    v2 = DeliveryPlanner_PartB.BOX_DOWN_COST + value[new_x][new_y]
                                
                                else:
                                    floor_cost = grid_costs[new_x][new_y]
                                    v2 = self.delta_cost[a] + floor_cost + value[new_x][new_y]

                                # print (v2)

                                if v2 < value[x][y]:
                                    change = True
                                    value[x][y] = v2
                                    policy[x][y] = "move " + self.delta_directions[a]   
                                if goal[0] == x and goal[1] == y and new_x>=0 and new_x<len(grid) and new_y>=0 and new_y<len(grid[0]):
                                    if value[new_x][new_y]<target_neighbors_min_value:
                                        target_neighbors_min_value = value[new_x][new_y]
                                        policy[x][y] = "move " + self.delta_directions[a]   
                                if new_y==goal[1] and new_x==goal[0]:
                                    policy[x][y] = "down " +  self.delta_directions[a] 
                                
                        else:
                            policy[x][y] = "-1"
                
       
        return policy

    def plan_delivery(self, debug=False):
        """
        plan_delivery() is required and will be called by the autograder directly.  
        You may not change the function signature for it.
        Add logic here to find the policies:  First to the box from any grid position
        then to the dropzone, again from any grid position.  You may use the starter
        code provided above in any way you choose, but please condition any printouts
        on the debug flag
        """
        ###########################################################################
        # Following is an example of how one could structure the solution using
        # the starter code we've provided.
        ###########################################################################

        # Start by finding a policy to direct the robot to the box from any grid position
        # The last command(s) in this policy will be 'lift 1' (i.e. lift box 1)
        goal = self.boxes['1']
        to_box_policy = self._find_policy(goal, pickup_box=True, debug=debug)

        # Now that the robot has the box, transition to the deliver policy.  The
        # last command(s) in this policy will be 'down x' where x = the appropriate
        # direction to set the box into the dropzone
        goal = self.dropzone
        deliver_policy = self._find_policy(goal, pickup_box=False, debug=debug)

        if debug:
            print("\nTo Box Policy:")
            for i in range(len(to_box_policy)):
                print(to_box_policy[i])

            print("\nDeliver Policy:")
            for i in range(len(deliver_policy)):
                print(deliver_policy[i])

        return (to_box_policy, deliver_policy) #


class DeliveryPlanner_PartC:
    """
    Required methods in this class are:

        plan_delivery(self, debug = False) which is stubbed out below.
        You may not change the method signature as it will be called directly
        by the autograder but you may modify the internals as needed.

        __init__: required to initialize the class.  Starter code is
        provided that initializes class variables based on the definitions in
        testing_suite_partC.py.  You may choose to use this starter code
        or modify and replace it based on your own solution

    The following methods are starter code you may use for part C.
    However, they are not required and can be replaced with your
    own methods.

        _set_initial_state_from(self, warehouse): creates structures based on
            the warehouse and todo definitions and initializes the robot
            location in the warehouse

        _find_policy(self, goal, pickup_box=True, debug=False): 
            Where the bulk of your algorithm could reside.
            It should find an optimal policy to a goal.
            Remember that actions are stochastic rather than deterministic.
            Hint:  you may want to structure this based
            on whether looking for a box or delivering a box.

    """

    # Definitions taken from testing_suite_partA.py
    ORTHOGONAL_MOVE_COST = 2
    DIAGONAL_MOVE_COST = 3
    BOX_LIFT_COST = 4
    BOX_DOWN_COST = 2
    ILLEGAL_MOVE_PENALTY = 100

    def __init__(self, warehouse, warehouse_cost, todo, stochastic_probabilities):

        self.todo = todo
        self.boxes_delivered = []
        self._set_initial_state_from(warehouse)
        self.warehouse_cost = warehouse_cost
        self.stochastic_probabilities = stochastic_probabilities

        self.delta = [
            [-1, 0],  # go up
            [-1, -1],  # up left (diag)
            [0, -1],  # go left
            [1, -1],  # dn left (diag)
            [1, 0],  # go down
            [1, 1],  # dn right (diag)
            [0, 1],  # go right
            [-1, 1],  # up right (diag)]
        ]

        self.delta_directions = ["n", "nw", "w", "sw", "s", "se", "e", "ne"]

        # Use this for a visual debug
        # self.delta_name = ['🡑', '🡔', '🡐', '🡗', '🡓', '🡖', '🡒', '🡕']

        # Costs for each move
        self.delta_cost = [self.ORTHOGONAL_MOVE_COST, self.DIAGONAL_MOVE_COST,
                           self.ORTHOGONAL_MOVE_COST, self.DIAGONAL_MOVE_COST,
                           self.ORTHOGONAL_MOVE_COST, self.DIAGONAL_MOVE_COST,
                           self.ORTHOGONAL_MOVE_COST, self.DIAGONAL_MOVE_COST, ]

    # state parsing and initialization function
    def _set_initial_state_from(self, warehouse):
        """Set initial state.

        Args:
            warehouse(list(list)): the warehouse map.
        """
        rows = len(warehouse)
        cols = len(warehouse[0])

        self.warehouse_state = [[None for j in range(cols)] for i in range(rows)]
        self.dropzone = None
        self.boxes = dict()

        for i in range(rows):
            for j in range(cols):
                this_square = warehouse[i][j]

                if this_square == '.':
                    self.warehouse_state[i][j] = '.'

                elif this_square == '#':
                    self.warehouse_state[i][j] = '#'

                elif this_square == '@':
                    self.warehouse_state[i][j] = '@'
                    self.dropzone = (i, j)

                else:  # a box
                    box_id = this_square
                    self.warehouse_state[i][j] = box_id
                    self.boxes[box_id] = (i, j)

    def _find_policy(self, goal, pickup_box=True, debug=False):
        """
        You are free to use any algorithm necessary to complete this task.
        Some algorithms may be more well suited than others, but deciding on the
        algorithm will allow you to think about the problem and understand what
        tools are (in)adequate to solve it. Please condition any printout on the
        debug flag provided in the argument. You may change this function signature
        (i.e. add arguments) as necessary, except for the debug argument which
        must remain with a default of False
        """

        ##############################################################################
        # insert code in this method if using the starter code we've provided
        ##############################################################################

        # get a shortcut variable for the warehouse (note this is just a view it does not make a copy)
        grid = self.warehouse_state
        grid_costs = self.warehouse_cost

        for col in range(len(grid[0])):
            for row in range(len(grid)):
                if self.warehouse_cost[row][col]=="w":
                    grid_costs[row][col] = float("inf")
        

        # You will need to fill in the algorithm here to find the policy
        # The following are what your algorithm should return for test case 1
        collision_cost = 100
        
        if pickup_box:
            # To box policy
            
            value = [[9999 for col in range(len(grid[0]))] for row in range(len(grid))]
            policy = [[" " for col in range(len(grid[0]))] for row in range(len(grid))]

            change = True
            
            while change:
                change = False
                for x in range(len(grid)):
                    for y in range(len(grid[0])):
                        if goal[0] == x and goal[1] == y:
                            if value[x][y] != 0:
                                value[x][y] = 0
                                policy[x][y] = "B"
                                change = True

                        elif grid[x][y] == "." or grid[x][y] == "@":
                            for a in range(len(self.delta)):
                                v2 = 0
                                
                                for i in range(-2, 3):
                                    a2 = (a + i) % len(self.delta)
                                    new_x = x + self.delta[a2][0]
                                    new_y = y + self.delta[a2][1]

                                    if i == 0:
                                        p2 = self.stochastic_probabilities["as_intended"]

                                    elif i==-2 or i == 2:
                                        p2 = (self.stochastic_probabilities["sideways"]) 
                                    else:
                                        p2 = (self.stochastic_probabilities["slanted"]) 
                                    if new_y>=len(grid[0]) or new_y<0 or new_x>=len(grid) or new_x<0 or grid[new_x][new_y]=="#":
                                        
                                        v2 += (self.delta_cost[a2] + collision_cost + value[x][y])*p2
                                    elif new_x==goal[0] and new_y==goal[1]:
                                        v2 += (value[new_x][new_y])*p2+DeliveryPlanner_PartC.BOX_LIFT_COST
                                    else:
                                        floor_cost = grid_costs[new_x][new_y]
                                        v2 += (self.delta_cost[a2] + floor_cost + value[new_x][new_y])*p2
                                
                            
                                    

                                if v2 < value[x][y]:
                                    change = True
                                    value[x][y] = v2
                                    policy[x][y] = "move " + self.delta_directions[a]   
                                if new_y==goal[1] and new_x==goal[0]:
                                    policy[x][y] = "lift " + grid[new_x][new_y]
                            
                        
                
                        else:
                            policy[x][y] = "-1"    
            
                
        else:
            # Deliver policy
            print(goal)
            value = [[9999 for col in range(len(grid[0]))] for row in range(len(grid))]
            policy = [[" " for col in range(len(grid[0]))] for row in range(len(grid))]

            change = True
            
            while change:
                change = False
                
                for x in range(len(grid)):
                    for y in range(len(grid[0])):
                        if goal[0] == x and goal[1] == y:
                            if value[x][y] != 0:
                                value[x][y] = 0                          
                                change = True

                        
                        elif grid[x][y] != "#":
                            for a in range(len(self.delta)):
                                v2 = 0
                                for i in range(-2, 3):
                                    a2 = (a + i) % len(self.delta)
                                    new_x = x + self.delta[a2][0]
                                    new_y = y + self.delta[a2][1]

                                    if i == 0:
                                        p2 = self.stochastic_probabilities["as_intended"]

                                    elif i==-2 or i == 2:
                                        p2 = (self.stochastic_probabilities["sideways"]) 
                                    else:
                                        p2 = (self.stochastic_probabilities["slanted"]) 
                                    if new_y>=len(grid[0]) or new_y<0 or new_x>=len(grid) or new_x<0 or grid[new_x][new_y]=="#":
                                        floor_cost = collision_cost
                                        v2 += (self.delta_cost[a2] + floor_cost + value[x][y])*p2
                                    elif new_x==goal[0] and new_y==goal[1]:
                                        v2 += ( value[new_x][new_y])*p2+DeliveryPlanner_PartC.BOX_DOWN_COST
                                    
                                    else:
                                        floor_cost = grid_costs[new_x][new_y]
                                        v2 += (self.delta_cost[a2] + floor_cost + value[new_x][new_y])*p2
                                

                                if v2 < value[x][y]:
                                    change = True
                                    value[x][y] = v2
                                    policy[x][y] = "move " + self.delta_directions[a]   
                                
                        else:
                            policy[x][y] = "-1"
            
            # for x in range(len(grid)):
            #     for y in range(len(grid[0])):
            #         if x==goal[0] and y==goal[1]:
            #         # if grid[x][y]!="#":
            #             neighbors_min_val = float("inf")
            #             for a in range(len(self.delta)):
            #                 new_x = goal[0] + self.delta[a][0]
            #                 new_y = goal[1] + self.delta[a][1]
                            
            #                 if new_x>=0 and new_x<len(grid) and new_y>=0 and new_y<len(grid[0]):
            #                     if value[new_x][new_y]<neighbors_min_val:
            #                         neighbors_min_val = value[new_x][new_y]
            #                         policy[goal[0]][goal[1]] = "move " + self.delta_directions[a]
                
            
            

            neighbors_min_val = float("inf")
            for a in range(len(self.delta)):
                new_x = goal[0] + self.delta[a][0]
                new_y = goal[1] + self.delta[a][1]
                if  new_x>=0 and new_x<len(grid) and new_y>=0 and new_y<len(grid[0]) and grid[new_x][new_y]!="#":    
                    policy[new_x][new_y] = "down " + self.delta_directions[self.delta.index([goal[0]-new_x, goal[1]-new_y])]
                    if value[new_x][new_y]<neighbors_min_val:
                        neighbors_min_val = value[new_x][new_y]
                        policy[goal[0]][goal[1]] = "move " + self.delta_directions[a]
                    
        print(value)

        return policy

    def plan_delivery(self, debug=False):
        """
        plan_delivery() is required and will be called by the autograder directly.
        You may not change the function signature for it.
        Add logic here to find the policies:  First to the box from any grid position
        then to the dropzone, again from any grid position.  You may use the starter
        code provided above in any way you choose, but please condition any printouts
        on the debug flag
        """
        ###########################################################################
        # Following is an example of how one could structure the solution using
        # the starter code we've provided.
        ###########################################################################

        # Start by finding a policy to direct the robot to the box from any grid position
        # The last command(s) in this policy will be 'lift 1' (i.e. lift box 1)
        goal = self.boxes['1']
        to_box_policy = self._find_policy(goal, pickup_box=True, debug=debug)

        # Now that the robot has the box, transition to the deliver policy.  The
        # last command(s) in this policy will be 'down x' where x = the appropriate
        # direction to set the box into the dropzone
        goal = self.dropzone
        to_zone_policy = self._find_policy(goal, pickup_box=False, debug=debug)

        if debug:
            print("\nTo Box Policy:")
            for i in range(len(to_box_policy)):
                print(to_box_policy[i])

            print("\nTo Zone Policy:")
            for i in range(len(to_zone_policy)):
                print(to_zone_policy[i])

        # For debugging purposes you may wish to return values associated with each policy.
        # Replace the default values of None with your grid of values below and turn on the
        # VERBOSE_FLAG in the testing suite.
        to_box_values = None
        to_zone_values = None
        return (to_box_policy, to_zone_policy, to_box_values, to_zone_values)


def who_am_i():
    # Please specify your GT login ID in the whoami variable (ex: jsmith223).
    whoami = 'lshamaei3'
    return whoami


if __name__ == "__main__":
    """ 
    You may execute this file to develop and test the search algorithm prior to running 
    the delivery planner in the testing suite.  Copy any test cases from the
    testing suite or make up your own.
    Run command:  python warehouse.py
    """

    # Test code in here will NOT be called by the autograder
    # This section is just a provided as a convenience to help in your development/debugging process

    # Testing for Part A
    # testcase 1
    print('\nTesting for part A:')

    from testing_suite_partA import wrap_warehouse_object, Counter

    # test case data starts here
    warehouse = [
        '######',
        '#....#',
        '#.1#2#',
        '#..#.#',
        '#...@#',
        '######',
    ]
    todo = list('12')
    benchmark_cost = 23
    viewed_cell_count_threshold = 20
    robot_position = (4,4)
    box_locations = {
        '1': (2,2),
        '2': (2,4),
    }
    
   
    # # # test case data ends here

    viewed_cells = Counter()
    warehouse_access = wrap_warehouse_object(warehouse, viewed_cells)
    partA = DeliveryPlanner_PartA(warehouse_access, robot_position, todo, box_locations)
    
    # partA._search(grid=warehouse, end=[4, 4], starting=[3, 2], obj='drop', box=1 ,debug=True)
    partA.plan_delivery(debug=True)
    # Note that the viewed cells for the hard coded solution provided
    # in the initial template code will be 0 because no actual search
    # process took place that accessed the warehouse
    print('Viewed Cells:', len(viewed_cells))
    print('Viewed Cell Count Threshold:', viewed_cell_count_threshold)

    # Testing for Part B
    # testcase 1
    # print('\nTesting for part B:')
    # warehouse = ['1..',
    #                             '.#.',
    #                             '..@']#['##.####1', '#.......', '@.......']

    # warehouse_cost =  [[3, 5, 2],
    #                                  [10, 'w', 2],
    #                                  [2, 10, 2]]
    # #[['w', 'w', 3, 'w', 'w', 'w', 'w', 12],
    #                     # ['w', 8, 10, 2, 10, 4, 15, 8],
    #                     # [15, 10, 10, 10, 7, 10, 2, 10]]

    # todo = ['1']

    # partB = DeliveryPlanner_PartB(warehouse, warehouse_cost, todo)
    # partB.plan_delivery(debug=True)

    # Testing for Part C
    # testcase 1
  