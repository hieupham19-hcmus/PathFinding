import numpy as np
import heapq

class GreedyBestFirstSearch:
    def __init__(self, map_instance, visualizer=None):
        self.map_instance = map_instance
        self.visualizer = visualizer

    def _move_cost(self, current, next):
        if current[0] != next[0] and current[1] != next[1]:  # Diagonal move
            return 1.5
        return 1  # Straight move

    def _reconstruct_path(self, came_from, start, goal):
        """Reconstruct the path from start to goal, including stops visited along the way."""
        current = goal
        path = []
        while current != start:
            if current not in self.map_instance.stops:  # Include only non-stop points in the path\
                self.map_instance.matrix[current[1]][current[0]] = 'X'  # Mark the path on the map
            path.append(current)
            current = came_from[current]
        path.append(start)  # Include the start point in the path
        path.reverse()  # Reverse the path to have it in correct order
        return path

    def _heuristic(self, a, b):
        return np.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

    def _neighbors(self, point):
        dirs = [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (1, -1), (-1, -1), (-1, 1)]  # Including diagonals
        result = []
        for d in dirs:
            next_point = (point[0] + d[0], point[1] + d[1])
            if 0 <= next_point[0] < self.map_instance.width and 0 <= next_point[1] < self.map_instance.height and \
                    self.map_instance.matrix[next_point[1]][
                        next_point[0]] != '*' and self.map_instance.matrix[next_point[1]][next_point[0]] != '#':
                # Check if the movement intersects any polygon
                intersects_polygon = False
                for polygon in self.map_instance.polygons:
                    if polygon.intersects_edge(point, next_point):  # Assumes implementation of this method in Polygon
                        intersects_polygon = True
                        break
                if not intersects_polygon:
                    result.append(next_point)
        return result
    def search_between_points(self,start, goal):
                frontier = []
                heapq.heappush(frontier, (0, start))  # Priority is just the heuristic
                came_from = {start: None}
                cost_so_far = {start: 0}

                # Visualization: Initialize open and closed sets for visualizer
                open_set = set([start])
                closed_set = set()
                while frontier:
                    current = heapq.heappop(frontier)[1]
                    closed_set.add(current)

                    if current == goal:
                        break

                    for next in self._neighbors(current):
                        new_cost = cost_so_far[current] + self._move_cost(current, next)


                        if next not in came_from or new_cost < cost_so_far.get(next, float('inf')):  # Check for better path
                            cost_so_far[next] = new_cost
                            priority = self._heuristic(next, goal)
                            heapq.heappush(frontier, (priority, next))
                            came_from[next] = current
                            open_set.add(next)
                    # Visualization: Update visualizer with current open and closed sets
                    if self.visualizer:
                        self.visualizer.visualize_search_step(list(open_set), list(closed_set), [])
                
                
                path = self._reconstruct_path(came_from, start, goal)
                if self.visualizer:
                    self.visualizer.update_visualization(path)
                return path, cost_so_far.get(goal, float('inf'))
    def _find_nearest_stop(self, current, stop_points):
        """Find the nearest stop point to the current point based on heuristic."""
        # This is a simplified version, in practice, you might calculate the actual distance or a heuristic value
        nearest_stop = min(stop_points, key=lambda stop: self._heuristic(current, stop))
        return nearest_stop

    def greedy_best_first_search(self):
        """
        Modified Greedy Best-First Search to include mandatory stops before reaching the final goal.
        """
        path_to_goal = []  # This will store the entire path including stops
        total_cost = 0
        current_start = self.map_instance.start_point
        stops = self.map_instance.stops.copy()  # Make a copy to avoid modifying the original list
        goal = self.map_instance.end_point

        # Function to perform Greedy Best-First Search between two points
        

        while stops:
            next_stop = self._find_nearest_stop(current_start, stops)
            path, cost = self.search_between_points(current_start, next_stop)
            if path is None:  # Path not found
                return None, float('inf')
            total_cost += cost
            path_to_goal.extend(path[:-1])  # Exclude the last point to avoid duplication
            current_start = next_stop
            stops.remove(next_stop)

        # Finally, go to the goal
        final_path, final_cost = self.search_between_points(current_start, goal)
        if final_path is None:
            return None, float('inf')
        total_cost += final_cost
        path_to_goal.extend(final_path)
        if self.visualizer:
            self.visualizer.update_visualization(path_to_goal)
        return path_to_goal, total_cost
