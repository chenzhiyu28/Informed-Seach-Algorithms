# Some parts of this code were adapted from COMP3702 Tutorial 3
# solution code ("tutorial3.py" by njc, available on COMP3702
# Blackboard page, retrieved 19 Aug 2022)


import sys
from constants import *
from environment import *
from state import State
import heapq


"""
solution.py

This file is a template you should use to implement your solution.

You should implement 

COMP3702 2022 Assignment 1 Support Code

Last updated by njc 01/08/22
"""


class StateNode:
    def __init__(self, env, state, parent, action_from_parent, path_steps,
                 path_cost):
        """
        :param env: environment
        :param state: state belonging to this node
        :param parent: parent of this node
        :param action_from_parent: LEFT, RIGHT, UP, or DOWN
        """
        self.env = env
        self.state = state  # an object of State class
        self.parent = parent
        self.action_from_parent = action_from_parent
        self.path_steps = path_steps
        self.path_cost = path_cost

        """
        self.state = State(self.env,
                           self.env.robot_init_posit,
                           self.env.robot_init_orient,
                           self.env.widget_init_posits,
                           self.env.widget_init_orients)
        """

    def get_path(self):
        """
        :return: A list of actions
        """
        path = []
        cur = self
        while cur.action_from_parent is not None:
            path.append(cur.action_from_parent)
            cur = cur.parent
        path.reverse()
        return path

    def get_successors(self):
        """
        :return: A list of successor StateNodes
        """
        successors = []
        ACTIONS = [FORWARD, REVERSE, SPIN_LEFT, SPIN_RIGHT]
        for a in ACTIONS:
            success, cost, next_state = self.env.perform_action(self.state, a)
            if success:
                successors.append(StateNode(self.env, next_state, self, a,
                                            self.path_steps + 1,
                                            self.path_cost + cost))
        return successors

    def __lt__(self, other):
        # compare nodes using path cost (for A*, this is overridden)
        return self.path_cost < other.path_cost


class Solver:

    def __init__(self, environment, loop_counter):
        self.environment = environment
        self.loop_counter = loop_counter
        self.verbose = True

        self.state = State(self.environment,
                           self.environment.robot_init_posit,
                           self.environment.robot_init_orient,
                           self.environment.widget_init_posits,
                           self.environment.widget_init_orients)
        #
        # TODO: Define any class instance variables you require here.
        #

    def solve_ucs(self):
        """
        Find a path which solves the environment using Uniform Cost Search (UCS).
        :return: path (list of actions, where each action is an element of ROBOT_ACTIONS)
        """

        #
        #
        # TODO: Implement your UCS code here
        #
        # === Important ================================================================================================
        # To ensure your code works correctly with tester, you should include
        # the following line of code in your main
        # search loop:
        #
        # self.loop_counter.inc()
        #
        # e.g.
        # while loop_condition():
        #   self.loop_counter.inc()
        #   ...
        #
        # ==============================================================================================================
        #
        #

        container = [StateNode(self.environment,
                               self.environment.get_init_state(),
                               None,
                               None,
                               0,
                               0)]

        heapq.heapify(container)

        # dict: state --> path_cost
        visited = {self.environment.get_init_state(): 0}
        n_expanded = 0

        while len(container) > 0:
            n_expanded += 1
            node = heapq.heappop(container)

            self.loop_counter.inc()
            """
            state = State(self.environment,
                          self.environment.robot_init_posit,
                          self.environment.robot_init_orient,
                          self.environment.widget_init_posits,
                          self.environment.widget_init_orients)
            """

            # print(node.state.robot_posit)
            # print(n_expanded)

            # check if this state is the goal
            if self.environment.is_solved(node.state):

                if self.verbose:
                    print(
                        f'Visited Nodes: {len(visited.keys())},'
                        f'\t\tExpanded Nodes: {n_expanded},\t\t'
                        f'Nodes in Container: {len(container)}')
                    print(
                        f'Cost of Path (with Costly Moves): {node.path_cost}')

                # print(node.env.render(node.state))
                return node.get_path()

            # add unvisited (or visited at higher path cost) successors to
            # container
            successors = node.get_successors()
            for s in successors:
                if s.state not in visited.keys() \
                        or s.path_cost < visited[s.state]:
                    visited[s.state] = s.path_cost
                    heapq.heappush(container, s)

        return None

    def calculate_distance(self,
                           widget_cells_separate,
                           target_list: list[tuple[int, int], ...]
                           ) -> int:
        total_distance = 0
        widget_centers = []
        widget_number = len(widget_cells_separate)

        # renter a list containing all center positions of widgets
        for widget_pos in widget_cells_separate:
            x = 0
            y = 0
            for pos in widget_pos:
                x = x + pos[0]
                y = y + pos[1]
            widget_centers.append((x/len(widget_pos), y/len(widget_pos)))

            distances = []
            for i in target_list:
                # current widget center location
                row = widget_centers[-1][0]
                col = widget_centers[-1][1]
                # get distance to the closest position
                distances.append(abs(i[0]-row) + abs(i[1]-col))
            distance = min(distances)/2
            total_distance = (total_distance + distance)

        return (10 * total_distance)/widget_number

    def Euclidean_distance(self,
                           widget_cells_separate,
                           target_list: list[tuple[int, int], ...]
                           ) -> int:

        total_distance = 0
        widget_centers = []
        widget_number = len(widget_cells_separate)

        # return a list containing all center positions of widgets
        for widget_pos in widget_cells_separate:
            x = 0
            y = 0
            for pos in widget_pos:
                x = x + pos[0]
                y = y + pos[1]
            widget_centers.append(
                (x / len(widget_pos), y / len(widget_pos)))

            distances = []
            for i in target_list:
                # current widget center location
                row = widget_centers[-1][0]
                col = widget_centers[-1][1]
                # get distance to the closest position
                distances.append(abs(i[0] - row) + abs(i[1] - col))
                distances.append(((i[0] - row) ^ 2 + (i[1] - col) ^ 2) ** 0.5)
            distance = min(distances) / 2
            total_distance = (total_distance + distance)

        return (5 * total_distance) / widget_number

    def dismath_rate(self, widget_cells_separate, target_list) -> float:
        widget_cells = []
        count = 0
        for widget_pos in widget_cells_separate:
            for pos in widget_pos:
                widget_cells.append(pos)

        for i in widget_cells:
            if i in target_list:
                count += 1

        return count/len(target_list)

    def heuristic(self, s) -> float:
        # s is a StateNode object
        # prioritize the state which make each widget center closer to target
        # Manhattan distance for center of widgets and target
        widget_cells_separate = []
        for i in range(s.env.n_widgets):
            widget_cell = widget_get_occupied_cells(
                s.env.widget_types[i],
                s.state.widget_centres[i],
                s.state.widget_orients[i])

            widget_cells_separate.append(widget_cell)

        target_list = s.env.target_list
        value = self.calculate_distance(widget_cells_separate, target_list)

        return value

    def solve_a_star(self):
        """
        Find a path which solves the environment using A* search.
        :return: path (list of actions, where each action is an element of ROBOT_ACTIONS)
        """

        #
        #
        # TODO: Implement your A* search code here
        #
        # === Important ================================================================================================
        # To ensure your code works correctly with tester, you should include the following line of code in your main
        # search loop:
        #
        # self.loop_counter.inc()
        #
        # e.g.
        # while loop_condition():
        #   self.loop_counter.inc()
        #   ...
        #
        # ==============================================================================================================
        #
        #

        # def a_star(env, heuristic, verbose=True):
        initial_StateNode = StateNode(self.environment,
                                      self.environment.get_init_state(),
                                      None,
                                      None,
                                      0,
                                      0)

        container = [(0 + self.heuristic(initial_StateNode),
                      initial_StateNode)]

        heapq.heapify(container)
        # dict: state --> path_cost
        visited = {self.environment.get_init_state(): 0}
        n_expanded = 0

        while len(container) > 0:
            n_expanded += 1
            _, node = heapq.heappop(container)

            self.loop_counter.inc()
            # check if this state is the goal
            if self.environment.is_solved(node.state):
                if self.verbose:
                    print(
                        f'Visited Nodes: {len(visited.keys())},'
                        f'\t\tExpanded Nodes: {n_expanded},\t\t'
                        f'Nodes in Container: {len(container)}')
                    print(
                        f'Cost of Path (with Costly Moves): {node.path_cost}')
                return node.get_path()

            # add unvisited (or visited at higher path cost) successors to container
            # s is 4 possible next StateNode instances, would all be added.
            successors = node.get_successors()
            for s in successors:
                if s.state not in visited.keys() or \
                        s.path_cost < visited[s.state]:
                    visited[s.state] = s.path_cost

                    # g(n) in [0, 1.8]
                    heapq.heappush(container,
                                   (s.path_cost + self.heuristic(s), s))

        return None

    #
    #
    # TODO: Add any additional methods here
    #
    #
