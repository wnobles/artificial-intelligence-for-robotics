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

import heapq
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
    Note: All print outs must be conditioned on the debug parameter.

    Required methods in this class are:

      plan_delivery(self, debug = False):
       Stubbed out below.  You may not change the method signature
        as it will be called directly by the autograder but you
        may modify the internals as needed.

      __init__:
        Required to initialize the class.  Signature can NOT be changed.
        Basic template starter code is provided.  You may choose to
        use this starter code or modify and replace it based on
        your own solution.
    """

    def __init__(self, warehouse_viewer, dropzone_location, todo, box_locations):

        self.warehouse_viewer = warehouse_viewer
        self.dropzone_location = dropzone_location
        self.todo = todo
        self.box_locations = box_locations

        # You may use these symbols indicating direction for visual debugging
        # ['^', '<', 'v', '>', '\\', '/', '[', ']']
        # or you may choose to use arrows instead
        # ['🡑', '🡐', '🡓', '🡒',  '🡔', '🡕', '🡖', '🡗']
        self.cost = {
            'move': {(-1, 0): 2, (-1, 1): 3, (0, 1): 2, (1, 1): 3, (1, 0): 2, (1, -1): 3, (0, -1): 2, (-1, -1): 3},
            'lift': 4,
            'down': 2}
        self.delta = [(-1, 0), (-1, 1), (0, 1), (1, 1), (1, 0), (1, -1), (0, -1), (-1, -1)]
        self.delta_name = ['n', 'ne', 'e', 'se', 's', 'sw', 'w', 'nw']

    def heuristic(self, x, y, goal):

        d_max = max(abs(x - goal[0]), abs(y - goal[1]))
        d_min = min(abs(x - goal[0]), abs(y - goal[1]))
        c_n = 2 # cost of non-diagonal movement
        c_d = 3 # cost of diagonal movement

        return c_d * d_min + c_n * (d_max - d_min)

    def search(self, init, goal, box, to_box):

        # The code below is adapted from:
        # Search: First Search Program (Uniform-Cost Search)
        # https://gatech.instructure.com/courses/402244/pages/9-first-search-program-uniform-cost-search?module_item_id=4089084
        # Search: Expansion Grid (Answer)
        # https://gatech.instructure.com/courses/402244/pages/10-expansion-grid-answer?module_item_id=4089090
        # Search: Print Path (Answer)
        # https://gatech.instructure.com/courses/402244/pages/11-print-path-answer?module_item_id=4089094
        # Search: Implement A* (Answer)
        # https://gatech.instructure.com/courses/402244/pages/13-implement-a-star-answer?module_item_id=4089100

        # initialize a closed list and add the start position
        closed_list = set()
        closed_list.add(tuple(init))

        # initialize a list to store the actions taken
        action = {}

        x, y = init
        g = 0
        h = self.heuristic(x, y, goal)
        f = g + h

        # initialize the open list to the initial coordinates and f, g, and h values
        open_list = [[f, g, h, x, y]]

        # convert the list to a heap
        heapq.heapify(open_list)

        found = False # flag to set if we reach the goal
        resign = False # flag to set if we can no longer expand

        while not found and not resign:

            # if no elements are on the open list, then we fail
            if len(open_list) == 0:
                resign = True

            #  otherwise, continue expanding
            else:

                # remove the node with the lowest f value
                next_node = heapq.heappop(open_list)

                # set the coordinates and cost
                f, g, h, x, y = next_node

                # if we're adjacent to the goal, then it's a success
                if abs(x - goal[0]) <= 1 and abs(y - goal[1]) <= 1:
                    found = True

                # otherwise, continue expanding
                for i, (dx, dy) in enumerate(self.delta):

                    # construct the next movements
                    x2 = x + dx
                    y2 = y + dy

                    # if the (x, y) values aren't checked and aren't an obstacle
                    if to_box:
                        obstacles = [b for b in self.todo if b != box] + ['#']
                    else:
                        obstacles = self.todo + ['#']
                    if (x2, y2) not in closed_list and self.warehouse_viewer[x2][y2] not in obstacles:

                        if (x2, y2) == goal:

                            if to_box:
                                # increment the lift cost
                                g2 = g + self.cost['lift']
                            else:
                                # increment the down cost
                                g2 = g + self.cost['down']

                        else:
                            # increment the move cost
                            g2 = g + self.cost['move'][(dx, dy)]

                        # calculate the heuristic, f score, and add to the open list
                        h2 = self.heuristic(x2, y2, goal)
                        f2 = g2 + h2
                        heapq.heappush(open_list, [f2, g2, h2, x2, y2])

                        # check off the (x, y) coordinate, add the index to the action
                        closed_list.add((x2, y2))
                        action[(x2, y2)] = i

        # initialize a dictionary to store the movements
        policy = {}

        # set the goal coordinates
        x, y = goal

        # iterate in reverse until we reach the initial coordinates
        while x != init[0] or y != init[1]:

            # get back the previous action and update the policy
            x2 = x - self.delta[action[(x, y)]][0]
            y2 = y - self.delta[action[(x, y)]][1]
            policy[(x2, y2)] = self.delta_name[action[(x, y)]]

            x = x2
            y = y2

        # reverse the policy
        policy = dict(reversed(policy.items()))

        # End of code citation

        return policy

    def plan_delivery(self, debug=False):
        """
        plan_delivery() is required and will be called by the autograder directly.
        You may not change the function signature for it.
        All print outs must be conditioned on the debug flag.
        """

        # The following is the hard coded solution to test case 1
        # moves = ['move w', 'move nw', 'lift 1', 'move se', 'down e', 'move ne', 'lift 2', 'down s']

        moves = []

        # set the initial location
        current_location = self.dropzone_location

        # iterate over the to-do list
        for box in self.todo:

            # get the path to the box
            path_to_box = self.search(current_location, self.box_locations[box], box, to_box=True)

            # update the warehouse viewer
            x, y = self.box_locations[box]
            self.warehouse_viewer[x][y] = '.'

            # update the last value of the dictionary
            last_key, last_value = list(path_to_box.items())[-1]
            path_to_box[last_key] = 'lift ' + box

            # append the list of movements
            moves.extend([f'move {d}' if not d.startswith('lift') else d for d in path_to_box.values()])

            # get the current location
            current_location = list(path_to_box.keys())[-1]

            # if we're standing on the drop zone...
            if current_location ==  self.dropzone_location:

                # add the last direction to the list of movements and set the next location to that of the box
                moves.extend([f'move {last_value}'])
                current_location = self.box_locations[box]

            # get the path from the box to the drop zone
            path_to_dropzone = self.search(current_location, self.dropzone_location, box, to_box=False)

            # if we're some distance from the drop zone...
            if path_to_dropzone:

                # update the last value of the dictionary
                last_key, last_value = list(path_to_dropzone.items())[-1]
                path_to_dropzone[last_key] = 'down ' + last_value

                # append the list of movements
                moves.extend([f'move {d}' if not d.startswith('down') else d for d in path_to_dropzone.values()])

                # get the current location
                current_location = list(path_to_dropzone.keys())[-1]

        if debug:
            for move in moves:
                print(move)

        return moves


class DeliveryPlanner_PartB:
    """
    Note: All print outs must be conditioned on the debug parameter.

    Required methods in this class are:

        generate_policies(self, debug = False):
         Stubbed out below. You may not change the method signature
         as it will be called directly by the autograder but you
         may modify the internals as needed.

        __init__:
         Required to initialize the class.  Signature can NOT be changed.
         Basic template starter code is provided.  You may choose to
         use this starter code or modify and replace it based on
         your own solution.

    The following method is starter code you may use.
    However, it is not required and can be replaced with your
    own method(s).

        _set_initial_state_from(self, warehouse):
         creates structures based on the warehouse map

    """

    def __init__(self, warehouse, warehouse_cost, todo):

        self._set_initial_state_from(warehouse)
        self.warehouse_cost = warehouse_cost
        self.todo = todo

        # You may use these symbols indicating direction for visual debugging
        # ['^', '<', 'v', '>', '\\', '/', '[', ']']
        # or you may choose to use arrows instead
        # ['🡑', '🡐', '🡓', '🡒',  '🡔', '🡕', '🡖', '🡗']
        self.cost = {
            'move': {(-1, 0): 2, (-1, 1): 3, (0, 1): 2, (1, 1): 3, (1, 0): 2, (1, -1): 3, (0, -1): 2, (-1, -1): 3},
            'lift': 4,
            'down': 2}
        self.delta = [(-1, 0), (-1, 1), (0, 1), (1, 1), (1, 0), (1, -1), (0, -1), (-1, -1)]
        self.delta_name = ['n', 'ne', 'e', 'se', 's', 'sw', 'w', 'nw']


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

    def compute_policy(self, goal, to_box=False):

        # The code below is adapted from:
        # Search: Optimum Policy (Answer)
        # https://gatech.instructure.com/courses/402244/pages/19-optimum-policy-answer?module_item_id=4089124

        value = [[999999 for col in range(len(self.warehouse_state[0]))] for row in range(len(self.warehouse_state))]
        policy = [['-1' for col in range(len(self.warehouse_state[0]))] for row in range(len(self.warehouse_state))]

        change = True
        while change:
            change = False

            for x in range(len(self.warehouse_state)):
                for y in range(len(self.warehouse_state[0])):

                    if goal == (x, y):
                        if value[x][y] > 0:
                            value[x][y] = 0
                            if to_box:
                                policy[x][y] = 'B'
                            else:
                                costs = []
                                for i, (dx, dy) in enumerate(self.delta):
                                    x2 = x + dx
                                    y2 = y + dy

                                    if 0 <= x2 < len(self.warehouse_state) and 0 <= y2 < len(self.warehouse_state[0]) and self.warehouse_state[x2][y2] != '#':
                                        v2 = value[x2][y2] + self.cost['move'][(dx, dy)] + self.warehouse_cost[x2][y2]
                                        policy[x][y] = 'move ' + self.delta_name[i]
                                        costs.append((v2, policy[x][y]))
                                policy[x][y] = min(costs)[1]
                            change = True

                    elif self.warehouse_state[x][y] != '#':
                        for i, (dx, dy) in enumerate(self.delta):
                            x2 = x + dx
                            y2 = y + dy

                            if 0 <= x2 < len(self.warehouse_state) and 0 <= y2 < len(self.warehouse_state[0]) and self.warehouse_state[x2][y2] != '#':

                                if self.warehouse_state[x2][y2] == '1' and to_box:
                                    v2 = value[x2][y2] + self.cost['lift'] + self.warehouse_cost[x2][y2]
                                elif self.warehouse_state[x2][y2] == '@' and not to_box:
                                    v2 = value[x2][y2] + self.cost['down'] + self.warehouse_cost[x2][y2]
                                else:
                                    v2 = value[x2][y2] + self.cost['move'][(dx, dy)] + self.warehouse_cost[x2][y2]

                                if v2 < value[x][y]:
                                    change = True
                                    value[x][y] = v2
                                    policy[x][y] = 'move ' + self.delta_name[i]

        # End of code citation

        return policy

    def generate_policies(self, debug=False):
        """
        generate_policies() is required and will be called by the autograder directly.
        You may not change the function signature for it.
        All print outs must be conditioned on the debug flag.
        """

        # The following is the hard coded solution to test case 1
        # to_box_policy = [['B', 'lift 1', 'move w'],
        #           ['lift 1', '-1', 'move nw'],
        #           ['move n', 'move nw', 'move n']]

        # deliver_policy = [['move e', 'move se', 'move s'],
        #           ['move ne', '-1', 'down s'],
        #           ['move e', 'down e', 'move n']]

        box_location = self.boxes[self.todo[0]]
        deliver_location = self.dropzone

        to_box_policy = self.compute_policy(box_location, to_box=True)

        # update the warehouse state
        self.warehouse_state[box_location[0]][box_location[1]] = '.'

        deliver_policy = self.compute_policy(deliver_location, to_box=False)

        for x in range(len(self.warehouse_state)):
            for y in range(len(self.warehouse_state[0])):
                for dx, dy in self.delta:
                    x2 = x + dx
                    y2 = y + dy

                    # add "lift" or "down" to goal locations
                    if 0 <= x2 < len(self.warehouse_state) and 0 <= y2 < len(self.warehouse_state[0]) and self.warehouse_state[x2][y2] != '#':
                        if (x, y) == box_location:
                            to_box_policy[x2][y2] = 'lift ' + self.todo[0]
                        if (x, y) == deliver_location:
                            deliver_policy[x2][y2] = 'down ' + deliver_policy[x2][y2].replace('move ', '')

        if debug:
            print("\nTo Box Policy:")
            for row in to_box_policy:
                print(row)

            print("\nDeliver Policy:")
            for row in deliver_policy:
                print(row)

        return (to_box_policy, deliver_policy)


class DeliveryPlanner_PartC:
    """
    [Doc string same as part B]
    Note: All print outs must be conditioned on the debug parameter.

    Required methods in this class are:

        generate_policies(self, debug = False):
         Stubbed out below. You may not change the method signature
         as it will be called directly by the autograder but you
         may modify the internals as needed.

        __init__:
         Required to initialize the class.  Signature can NOT be changed.
         Basic template starter code is provided.  You may choose to
         use this starter code or modify and replace it based on
         your own solution.

    The following method is starter code you may use.
    However, it is not required and can be replaced with your
    own method(s).

        _set_initial_state_from(self, warehouse):
         creates structures based on the warehouse map

    """

    def __init__(self, warehouse, warehouse_cost, todo, stochastic_probabilities):

        self._set_initial_state_from(warehouse)
        self.warehouse_cost = warehouse_cost
        self.todo = todo
        self.stochastic_probabilities = stochastic_probabilities

        # You may use these symbols indicating direction for visual debugging
        # ['^', '<', 'v', '>', '\\', '/', '[', ']']
        # or you may choose to use arrows instead
        # ['🡑', '🡐', '🡓', '🡒',  '🡔', '🡕', '🡖', '🡗']
        self.cost = {
            'move': {(-1, 0): 2, (-1, 1): 3, (0, 1): 2, (1, 1): 3, (1, 0): 2, (1, -1): 3, (0, -1): 2, (-1, -1): 3},
            'lift': 4,
            'down': 2}
        self.delta = [(-1, 0), (-1, 1), (0, 1), (1, 1), (1, 0), (1, -1), (0, -1), (-1, -1)]
        self.delta_name = ['n', 'ne', 'e', 'se', 's', 'sw', 'w', 'nw']

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

    def stochastic_motion(self, goal, to_box=False):

        collision_cost = 100
        value = [[999999 for col in range(len(self.warehouse_state[0]))] for row in range(len(self.warehouse_state))]
        policy = [['-1' for col in range(len(self.warehouse_state[0]))] for row in range(len(self.warehouse_state))]

        # The following code is adapted from:
        # https://gatech.instructure.com/courses/402244/pages/5-stochastic-motion-answer?module_item_id=4089160
        # Problem Set 4: 5. Stochastic Motion (Answer)

        change = True
        while change:
            change = False

            for x in range(len(self.warehouse_state)):
                for y in range(len(self.warehouse_state[0])):

                    if goal == (x, y):
                        if value[x][y] > 0:
                            value[x][y] = 0
                            if to_box:
                                policy[x][y] = 'B'
                            else:
                                costs = []
                                for i, (dx, dy) in enumerate(self.delta):
                                    x2 = x + dx
                                    y2 = y + dy

                                    if 0 <= x2 < len(self.warehouse_state) and 0 <= y2 < len(self.warehouse_state[0]) and self.warehouse_state[x2][y2] != '#':
                                        v2 = value[x2][y2] + self.cost['move'][(dx, dy)] + self.warehouse_cost[x2][y2]
                                        policy[x][y] = 'move ' + self.delta_name[i]
                                        costs.append((v2, policy[x][y]))
                                policy[x][y] = min(costs)[1]
                            change = True

                    elif self.warehouse_state[x][y] != '#':
                        for a, (dx, dy) in enumerate(self.delta):
                            x2 = x + dx
                            y2 = y + dy

                            if 0 <= x2 < len(self.warehouse_state) and 0 <= y2 < len(self.warehouse_state[0]) and self.warehouse_state[x2][y2] != '#':
                                if self.warehouse_state[x2][y2] == '1' and to_box:
                                    v2 = value[x2][y2] + self.cost['lift'] + self.warehouse_cost[x2][y2]

                                elif self.warehouse_state[x2][y2] == '@' and not to_box:
                                    v2 = value[x2][y2] + self.cost['down'] + self.warehouse_cost[x2][y2]

                                else:
                                    v2 = 0
                                    for i in range(-2, 3):
                                        a2 = (a + i) % len(self.delta)
                                        dx2 = self.delta[a2][0]
                                        dy2 = self.delta[a2][1]
                                        x3 = x + dx2
                                        y3 = y + dy2

                                        if i == 0:
                                            p2 = self.stochastic_probabilities['as_intended']
                                        elif i in [-1, 1]:
                                            p2 = self.stochastic_probabilities['slanted']
                                        else:
                                            p2 = self.stochastic_probabilities['sideways']

                                        if 0 <= x3 < len(self.warehouse_state) and 0 <= y3 < len(self.warehouse_state[0]) and self.warehouse_state[x3][y3] != '#':
                                            v2 += (value[x3][y3] + self.cost['move'][(dx2, dy2)] + self.warehouse_cost[x3][y3]) * p2
                                        else:
                                            v2 += (value[x][y] + self.cost['move'][(dx2, dy2)] + collision_cost) * p2

                                if v2 < value[x][y]:
                                    change = True
                                    value[x][y] = v2
                                    policy[x][y] = 'move ' + self.delta_name[a]

        # End of code citation

        return value, policy

    def generate_policies(self, debug=False):
        """
        generate_policies() is required and will be called by the autograder directly.
        You may not change the function signature for it.
        All print outs must be conditioned on the debug flag.
        """

        # The following is the hard coded solution to test case 1
        # to_box_policy = [
        #     ['B', 'lift 1', 'move w'],
        #     ['lift 1', -1, 'move nw'],
        #     ['move n', 'move nw', 'move n'],]

        # to_zone_policy = [
        #     ['move e', 'move se', 'move s'],
        #     ['move se', -1, 'down s'],
        #     ['move e', 'down e', 'move n'],]

        box_location = self.boxes[self.todo[0]]
        deliver_location = self.dropzone

        to_box_values, to_box_policy = self.stochastic_motion(box_location, to_box=True)

        # update the warehouse state
        self.warehouse_state[box_location[0]][box_location[1]] = '.'

        to_zone_values, to_zone_policy = self.stochastic_motion(deliver_location, to_box=False)

        for x in range(len(self.warehouse_state)):
            for y in range(len(self.warehouse_state[0])):
                for dx, dy in self.delta:
                    x2 = x + dx
                    y2 = y + dy

                    # add "lift" or "down" to goal locations
                    if 0 <= x2 < len(self.warehouse_state) and 0 <= y2 < len(self.warehouse_state[0]) and self.warehouse_state[x2][y2] != '#':
                        if (x, y) == box_location:
                            to_box_policy[x2][y2] = 'lift ' + self.todo[0]
                        if (x, y) == deliver_location:
                            to_zone_policy[x2][y2] = 'down ' + to_zone_policy[x2][y2].replace('move ', '')

        if debug:
            print("\nTo Box Policy:")
            for row in to_box_policy:
                print(row)

            print("\nTo Zone Policy:")
            for row in to_zone_policy:
                print(row)

        # For debugging purposes you may wish to return values associated with each policy.
        # Replace the default values of None with your grid of values below and turn on the
        # VERBOSE_FLAG in the testing suite.
        # to_box_values = None
        # to_zone_values = None

        return (to_box_policy, to_zone_policy, to_box_values, to_zone_values)


def who_am_i():
    # Please specify your GT login ID in the whoami variable (ex: jsmith224).
    whoami = 'wnobles6'
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
    print('\n~~~ Testing for part A: ~~~\n')

    from testing_suite_partA import wrap_warehouse_object, Counter

    # test case data starts here
    # testcase 1
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
    dropzone = (4,4)
    box_locations = {
        '1': (2,2),
        '2': (2,4),
    }
    # test case data ends here

    viewed_cells = Counter()
    warehouse_access = wrap_warehouse_object(warehouse, viewed_cells)
    partA = DeliveryPlanner_PartA(warehouse_access, dropzone, todo, box_locations)
    partA.plan_delivery(debug=True)
    # Note that the viewed cells for the hard coded solution provided
    # in the initial template code will be 0 because no actual search
    # process took place that accessed the warehouse
    print('Viewed Cells:', len(viewed_cells))
    print('Viewed Cell Count Threshold:', viewed_cell_count_threshold)

    # Testing for Part B
    # testcase 1
    print('\n~~~ Testing for part B: ~~~')
    warehouse = ['1..',
                 '.#.',
                 '..@']

    warehouse_cost = [[3, 5, 2],
                      [10, math.inf, 2],
                      [2, 10, 2]]

    todo = ['1']

    partB = DeliveryPlanner_PartB(warehouse, warehouse_cost, todo)
    partB.generate_policies(debug=True)

    # Testing for Part C
    # testcase 1
    print('\n~~~ Testing for part C: ~~~')
    warehouse = ['1..',
                 '.#.',
                 '..@']

    warehouse_cost = [[13, 5, 6],
                      [10, math.inf, 2],
                      [2, 11, 2]]

    todo = ['1']

    # stochastic_probabilities = {
    #     'as_intended': .70,
    #     'slanted': .1,
    #     'sideways': .05,
    # }
    stochastic_probabilities = {'as_intended': 0.2, 'slanted': 0.26666666666666666, 'sideways': 0.13333333333333333}

    partC = DeliveryPlanner_PartC(warehouse, warehouse_cost, todo, stochastic_probabilities)
    partC.generate_policies(debug=True)
