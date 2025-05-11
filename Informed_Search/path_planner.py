from grid import Node, NodeGrid
from math import inf
import heapq


class PathPlanner(object):
    """
    Represents a path planner, which may use Dijkstra, Greedy Search or A* to plan a path.
    """
    def __init__(self, cost_map):
        """
        Creates a new path planner for a given cost map.

        :param cost_map: cost used in this path planner.
        :type cost_map: CostMap.
        """
        self.cost_map = cost_map
        self.node_grid = NodeGrid(cost_map)

    @staticmethod
    def construct_path(goal_node):
        """
        Extracts the path after a planning was executed.

        :param goal_node: node of the grid where the goal was found.
        :type goal_node: Node.
        :return: the path as a sequence of (x, y) positions: [(x1,y1),(x2,y2),(x3,y3),...,(xn,yn)].
        :rtype: list of tuples.
        """
        node = goal_node
        # Since we are going from the goal node to the start node following the parents, we
        # are transversing the path in reverse
        reversed_path = []
        while node is not None:
            reversed_path.append(node.get_position())
            node = node.parent
        return reversed_path[::-1]  # This syntax creates the reverse list

    def dijkstra(self, start_position, goal_position):
        """
        Plans a path using the Dijkstra algorithm.

        :param start_position: position where the planning stars as a tuple (x, y).
        :type start_position: tuple.
        :param goal_position: goal position of the planning as a tuple (x, y).
        :type goal_position: tuple.
        :return: the path as a sequence of positions and the path cost.
        :rtype: list of tuples and float.
        """
		# Todo: implement the Dijkstra algorithm
		# The first return is the path as sequence of tuples (as returned by the method construct_path())
		# The second return is the cost of the path
        start_node = self.node_grid.get_node(start_position[0], start_position[1])
        goal_node = self.node_grid.get_node(goal_position[0], goal_position[1])

        start_node.g = 0

        pq = []
        heapq.heappush(pq, (start_node.g, start_node))

        while pq:
            current_cost, current_node = heapq.heappop(pq)
            current_node.closed = True

            if current_node == goal_node:
                path = self.construct_path(goal_node)
                self.node_grid.reset()
                return path, current_cost

            for sucessor_coordinates in self.node_grid.get_successors(current_node.i, current_node.j):
                sucessor_node = self.node_grid.get_node(sucessor_coordinates[0], sucessor_coordinates[1])
                sucessor_cost = self.cost_map.get_edge_cost(current_node.get_position(), sucessor_node.get_position())
                if (sucessor_node.closed is False) and (sucessor_node.g > current_cost + sucessor_cost):
                    sucessor_node.g = current_cost + sucessor_cost
                    sucessor_node.parent = current_node
                    heapq.heappush(pq, (sucessor_node.g, sucessor_node))

    def greedy(self, start_position, goal_position):
        """
        Plans a path using greedy search.

        :param start_position: position where the planning stars as a tuple (x, y).
        :type start_position: tuple.
        :param goal_position: goal position of the planning as a tuple (x, y).
        :type goal_position: tuple.
        :return: the path as a sequence of positions and the path cost.
        :rtype: list of tuples and float.
        """
		# Todo: implement the Greedy Search algorithm
		# The first return is the path as sequence of tuples (as returned by the method construct_path())
		# The second return is the cost of the path
        start_node = self.node_grid.get_node(start_position[0], start_position[1])
        goal_node = self.node_grid.get_node(goal_position[0], goal_position[1])

        start_node.g = 0
        start_node.f = start_node.distance_to(goal_node.i, goal_node.j)

        pq = []
        heapq.heappush(pq, (start_node.f, start_node))

        while pq:
            _, current_node = heapq.heappop(pq)
            current_node.closed = True

            if current_node == goal_node:
                path = self.construct_path(goal_node)
                self.node_grid.reset()
                return path, current_node.g

            for sucessor_coordinates in self.node_grid.get_successors(current_node.i, current_node.j):
                sucessor_node = self.node_grid.get_node(sucessor_coordinates[0], sucessor_coordinates[1])
                sucessor_cost = self.cost_map.get_edge_cost(current_node.get_position(), sucessor_node.get_position())
                if (sucessor_node.closed is False) and (sucessor_node.g > current_node.g + sucessor_cost):
                    sucessor_node.g = current_node.g + sucessor_cost
                    sucessor_node.f = sucessor_node.distance_to(goal_node.i, goal_node.j)
                    sucessor_node.parent = current_node
                    heapq.heappush(pq, (sucessor_node.f, sucessor_node))


    def a_star(self, start_position, goal_position):
        """
        Plans a path using A*.

        :param start_position: position where the planning stars as a tuple (x, y).
        :type start_position: tuple.
        :param goal_position: goal position of the planning as a tuple (x, y).
        :type goal_position: tuple.
        :return: the path as a sequence of positions and the path cost.
        :rtype: list of tuples and float.
        """
		# Todo: implement the A* algorithm
		# The first return is the path as sequence of tuples (as returned by the method construct_path())
		# The second return is the cost of the path
        start_node = self.node_grid.get_node(start_position[0], start_position[1])
        goal_node = self.node_grid.get_node(goal_position[0], goal_position[1])

        start_node.g = 0
        start_node.f = start_node.distance_to(goal_node.i, goal_node.j)

        pq = []
        heapq.heappush(pq, (start_node.f, start_node))

        while pq:
            _, current_node = heapq.heappop(pq)
            current_node.closed = True

            if current_node == goal_node:
                path = self.construct_path(goal_node)
                self.node_grid.reset()
                return path, current_node.g

            for sucessor_coordinates in self.node_grid.get_successors(current_node.i, current_node.j):
                sucessor_node = self.node_grid.get_node(sucessor_coordinates[0], sucessor_coordinates[1])
                sucessor_cost = self.cost_map.get_edge_cost(current_node.get_position(), sucessor_node.get_position())
                if (sucessor_node.closed is False) and (sucessor_node.g > current_node.g + sucessor_cost):
                    sucessor_node.g = current_node.g + sucessor_cost
                    sucessor_node.f = sucessor_node.g + sucessor_node.distance_to(goal_node.i, goal_node.j)
                    sucessor_node.parent = current_node
                    heapq.heappush(pq, (sucessor_node.f, sucessor_node))
        return [], inf
