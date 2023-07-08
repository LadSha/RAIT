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


    def _search(self, debug=False):
        """
        This method should be based on lesson modules for A*, see Search, Section 12-14.
        The bulk of the search logic should reside here, should you choose to use this starter code.
        Please condition any printout on the debug flag provided in the argument.  
        You may change this function signature (i.e. add arguments) as 
        necessary, except for the debug argument which must remain with a default of False
        """

        # get a shortcut variable for the warehouse (note this is just a view no copying)
        warehouse = self.warehouse_viewer

        # Find and fill in the required moves per the instructions - example moves for test case 1
        moves = ['move w',
                 'move nw',
                 'lift 1',
                 'move se',
                 'down e',
                 'move ne',
                 'lift 2',
                 'down s']

        return moves

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

        moves = self._search(debug=debug)

        if debug:
            for i in range(len(moves)):
                print(moves[i])

        return moves


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
                                else:
                                    floor_cost = value[new_x][new_y]
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
                                else:
                                    floor_cost = value[new_x][new_y]
                                    v2 = self.delta_cost[a] + floor_cost + value[new_x][new_y]

                                # print (v2)

                                if v2 < value[x][y]:
                                    change = True
                                    value[x][y] = v2
                                    policy[x][y] = "move " + self.delta_directions[a]   
                                if new_y==goal[1] and new_x==goal[0]:
                                    policy[x][y] = "down " + grid[new_x][new_y] 
                                
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
        self.delta_name = ['🡑', '🡔', '🡐', '🡗', '🡓', '🡖', '🡒', '🡕']

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

        # You will need to fill in the algorithm here to find the policy
        # The following are what your algorithm should return for test case 1
        if pickup_box:
            # To-box policy
            # the below policy is hard coded to work for test case 1
            policy = [
                ['B', 'lift 1', 'move w'],
                ['lift 1', -1, 'move nw'],
                ['move n', 'move nw', 'move n'],
            ]

        else:
            # to-zone policy
            # the below policy is hard coded to work for test case 1
            policy = [
                ['move e', 'move se', 'move s'],
                ['move se', -1, 'down s'],
                ['move e', 'down e', 'move n'],
            ]

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
    # print('\nTesting for part A:')

    # from testing_suite_partA import wrap_warehouse_object, Counter

    # # test case data starts here
    # warehouse = [
    #     '######',
    #     '#....#',
    #     '#.1#2#',
    #     '#..#.#',
    #     '#...@#',
    #     '######',
    # ]
    # todo = list('12')
    # benchmark_cost = 23
    # viewed_cell_count_threshold = 20
    # robot_position = (4,4)
    # box_locations = {
    #     '1': (2,2),
    #     '2': (2,4),
    # }
    # # test case data ends here

    # viewed_cells = Counter()
    # warehouse_access = wrap_warehouse_object(warehouse, viewed_cells)
    # partA = DeliveryPlanner_PartA(warehouse_access, robot_position, todo, box_locations)
    # partA.plan_delivery(debug=True)
    # # Note that the viewed cells for the hard coded solution provided
    # # in the initial template code will be 0 because no actual search
    # # process took place that accessed the warehouse
    # print('Viewed Cells:', len(viewed_cells))
    # print('Viewed Cell Count Threshold:', viewed_cell_count_threshold)

    # Testing for Part B
    # testcase 1
    print('\nTesting for part B:')
    warehouse = ['1..',
                                '.#.',
                                '..@']#['##.####1', '#.......', '@.......']

    warehouse_cost =  [[3, 5, 2],
                                     [10, 'w', 2],
                                     [2, 10, 2]]
    #[['w', 'w', 3, 'w', 'w', 'w', 'w', 12],
                        # ['w', 8, 10, 2, 10, 4, 15, 8],
                        # [15, 10, 10, 10, 7, 10, 2, 10]]

    todo = ['1']

    partB = DeliveryPlanner_PartB(warehouse, warehouse_cost, todo)
    partB.plan_delivery(debug=True)

    # Testing for Part C
    # testcase 1
    # print('\nTesting for part C:')
    # warehouse = ['1..',
    #              '.#.',
    #              '..@']

    # warehouse_cost = [[13, 5, 6],
    #                   [10, math.inf, 2],
    #                   [2, 11, 2]]

    # todo = ['1']

    # stochastic_probabilities = {
    #     'as_intended': .70,
    #     'slanted': .1,
    #     'sideways': .05,
    # }

    # partC = DeliveryPlanner_PartC(warehouse, warehouse_cost, todo, stochastic_probabilities)
    # partC.plan_delivery(debug=True)
