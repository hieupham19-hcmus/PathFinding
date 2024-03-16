import heapq
import numpy as np
import random
import pygame


class AStarDynamic:
    def __init__(self, map_instance, visualizer=None, step_threshold=10):
        self.map_instance = map_instance
        self.visualizer = visualizer
        self.step_threshold = step_threshold

    def _move_cost(self, current, next):
        # Diagonal move check simplified for readability
        return 1.5 if current[0] != next[0] and current[1] != next[1] else 1

    def _reconstruct_path(self, came_from, start, goal):
        path = []
        current = goal
        while current != start:
            # Marking the path on the map if not a stop point
            if current not in self.map_instance.stops:
                self.map_instance.matrix[current[1]][current[0]] = 'X'
            path.append(current)
            current = came_from[current]
        path.append(start)  # Adding start point to the path
        return path[::-1]  # Reversing path to start-to-goal order

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

    def _find_nearest_stop(self, current, stop_points):
        nearest_stop = min(stop_points, key=lambda stop: self._heuristic(current, stop))
        return nearest_stop

    # setting up the dynamic A* search, after step_thetareshhold steps, the polygon will be moved

    def a_star_search(self):
        path_to_goal = []
        total_cost = 0

        current_start = self.map_instance.start_point
        stops = self.map_instance.stops.copy()
        goal = self.map_instance.end_point

        while stops:
            next_stop = self._find_nearest_stop(current_start, stops)
            path, cost = self._dynamic_a_star_search_between_points(current_start, next_stop)
            total_cost += cost
            path_to_goal.extend(path[:-1])
            current_start = next_stop
            stops.remove(next_stop)

        final_path, final_cost = self._dynamic_a_star_search_between_points(current_start, goal)
        total_cost += final_cost
        path_to_goal.extend(final_path)

        if self.visualizer:
            random_color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
            self.visualizer.update_visualization(random_color, path_to_goal, total_cost)

        return path_to_goal, total_cost

    # use move_plygons(index) to move polygon and index is the index of robot that is moving
    def _dynamic_a_star_search_between_points(self, start, goal):
        frontier = []
        heapq.heappush(frontier, (0, start))
        came_from = {start: None}
        cost_so_far = {start: 0}
        steps_since_last_update = 0

        while frontier:
            current_priority, current = heapq.heappop(frontier)

            if current == goal:
                break

            for next in self._neighbors(current):
                new_cost = cost_so_far[current] + self._move_cost(current, next)
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + self._heuristic(goal, next)
                    heapq.heappush(frontier, (priority, next))
                    came_from[next] = current

            steps_since_last_update += 1
            if steps_since_last_update % self.step_threshold == 0:
                self.map_instance.move_polygons(current)
                if self.visualizer:
                    # Assuming you have a method in visualizer to clear and redraw everything.
                    # This redraws the entire state including moved polygons and the current path.
                    current_path = self._reconstruct_path(came_from, start, current)
                    self.visualizer.visualize_after_move_polygon(current_path, self.map_instance.polygons)
                    pygame.time.wait(200)

        if self.visualizer:
            # Final path visualization after the complete search
            final_path = self._reconstruct_path(came_from, start, goal)
            self.visualizer.visualize_after_move_polygon(final_path, self.map_instance.polygons)

        return self._reconstruct_path(came_from, start, goal), cost_so_far[goal]
