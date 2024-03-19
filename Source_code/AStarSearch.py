import heapq
import numpy as np
import random
import time


class AStarSearch:
    def __init__(self, map_instance, visualizer=None):
        self.map_instance = map_instance
        self.visualizer = visualizer

    def _move_cost(self, current, next):
        if current[0] != next[0] and current[1] != next[1]:  # Diagonal move
            return 1.5
        return 1  # Straight move

    def _reconstruct_path(self, came_from, start, goal):
        current = goal
        path = []
        while current != start:
            if current not in self.map_instance.stops:
                self.map_instance.matrix[current[1]][current[0]] = 'X'
            path.append(current)
            current = came_from[current]
        path.append(start)
        path.reverse()
        return path

    def _heuristic(self, a, b):
        return np.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

    def _neighbors(self, point):
        dirs = [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (1, -1), (-1, -1), (-1, 1)]
        result = []
        for d in dirs:
            next_point = (point[0] + d[0], point[1] + d[1])
            if 0 <= next_point[0] < self.map_instance.width and 0 <= next_point[1] < self.map_instance.height and \
                    self.map_instance.matrix[next_point[1]][next_point[0]] not in ['*', '#']:
                intersects_polygon = False
                for polygon in self.map_instance.polygons:
                    if polygon.intersects_edge(point, next_point):
                        intersects_polygon = True
                        break
                if not intersects_polygon:
                    result.append(next_point)
        return result

    def _a_star_search_between_points(self, start, goal):
        frontier = []
        heapq.heappush(frontier, (0, start))
        came_from = {start: None}
        cost_so_far = {start: 0}

        while frontier:
            current_priority, current = heapq.heappop(frontier)

            if current == goal:
                break

            for next in self._neighbors(current):
                new_cost = cost_so_far[current] + self._move_cost(current, next)
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + self._heuristic(next, goal)
                    heapq.heappush(frontier, (priority, next))
                    came_from[next] = current

        # Check if goal was never reached
        if goal not in came_from:
            return None, float('inf')  # Indicate no path was found

        path = self._reconstruct_path(came_from, start, goal)

        return path, cost_so_far[goal]

    def find_best_path(self, current_start, stops, goal, path_to_goal=None, total_cost=0,
                       best_result=None):
        if best_result is None:
            best_result = [float('inf'), []]
        if path_to_goal is None:
            path_to_goal = []

        if not stops:
            final_path, final_cost = self._a_star_search_between_points(current_start, goal)
            if final_path is None:  # No path found
                return  # Skip if no path to goal
            total_cost += final_cost
            if total_cost < best_result[0]:
                best_result[0] = total_cost
                best_result[1] = path_to_goal + final_path
        else:
            for i, stop in enumerate(stops):
                next_stops = stops[:i] + stops[i + 1:]
                path, cost = self._a_star_search_between_points(current_start, stop)
                if path is None:  # Skip if no path to this stop
                    continue
                if total_cost + cost >= best_result[0]:
                    continue
                self.find_best_path(stop, next_stops, goal, path_to_goal + path, total_cost + cost, best_result)

    def a_star_search(self):
        self.visualizer.draw_grid() if self.visualizer else None
        path_to_goal = []
        total_cost = 0

        current_start = self.map_instance.start_point
        stops = self.map_instance.stops.copy()
        goal = self.map_instance.end_point

        best_result = [float('inf'), []]
        self.find_best_path(current_start, stops, goal, path_to_goal, total_cost, best_result=best_result)

        if best_result[0] == float('inf'):
            self.visualizer.no_path_found()
            return None, float('inf')

        if self.visualizer:
            random_color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
            self.visualizer.update_visualization(random_color, best_result[1], best_result[0])

        return best_result[1], best_result[0]

