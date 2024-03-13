import heapq
import numpy as np


class AStarDynamic:
    def __init__(self, map_instance, visualizer=None, step_threshold=10):
        self.map_instance = map_instance
        self.visualizer = visualizer
        self.step_threshold = step_threshold

    def _move_cost(self, current, next):
        if current[0] != next[0] and current[1] != next[1]:
            return 1.5
        return 1

    def _heuristic(self, a, b):
        return np.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

    def _neighbors(self, point):
        dirs = [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (1, -1), (-1, -1), (-1, 1)]
        result = []
        for d in dirs:
            next_point = (point[0] + d[0], point[1] + d[1])
            if 0 <= next_point[0] < self.map_instance.width and 0 <= next_point[1] < self.map_instance.height and \
                    self.map_instance.matrix[next_point[1]][next_point[0]] not in ['*', '#']:
                result.append(next_point)
        return result

    def _search_between_points(self, start, goal, path_to_goal):
        frontier = []
        heapq.heappush(frontier, (0, start))
        came_from = {start: None}
        cost_so_far = {start: 0}
        open_set = set()
        closed_set = set()

        while frontier:
            current_priority, current = heapq.heappop(frontier)
            closed_set.add(current)

            if current == goal:
                break

            for next in self._neighbors(current):
                new_cost = cost_so_far[current] + self._move_cost(current, next)
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + self._heuristic(next, goal)
                    heapq.heappush(frontier, (priority, next))
                    came_from[next] = current

        return self._reconstruct_path(came_from, start, goal), cost_so_far.get(goal, 0)

    def _reconstruct_path(self, came_from, start, goal):
        current = goal
        path = []
        while current != start:
            path.append(current)
            current = came_from[current]
        path.append(start)
        path.reverse()
        return path

    def a_star_search(self):
        path_to_goal = []
        total_cost = 0
        current = self.map_instance.start_point
        goal = self.map_instance.end_point

        while current != self.map_instance.end_point:
            # Attempt to find a path to the next stop or the goal
            path, cost = self._search_between_points(current, self.map_instance.end_point, path_to_goal)
            

            total_cost += cost

            if path:
                # Update path if valid
                path_to_goal.extend(path if not path_to_goal else path[1:])  # Skip duplicate start node
                current = path_to_goal[-1]

                # Move polygons and check if the path is still valid
                self.map_instance.move_polygons(path_to_goal)

                # If the move made the current path invalid, clear it and recalculate
                if any(self.map_instance.polygons[i].intersects_edge(path_to_goal[j], path_to_goal[j + 1]) for i in
                       range(len(self.map_instance.polygons)) for j in range(len(path_to_goal) - 1)):
                    path_to_goal = []
                    current = self.map_instance.start_point
                    total_cost = 0
                    continue
            else:
                # No path found, cannot reach the end point from here
                break

        if self.visualizer:
            self.visualizer.update_visualization(path_to_goal)

        return path_to_goal, total_cost
