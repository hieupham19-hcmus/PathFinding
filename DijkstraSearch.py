import heapq


class DijkstraSearch:
    def __init__(self, map_instance, visualizer=None):
        self.map_instance = map_instance
        self.visualizer = visualizer

    def _move_cost(self, current, next):
        """Calculates the cost of moving from the current node to the next node."""
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

    def _mark_path_on_graph(self, path):
        """Đánh dấu đường đi đã chọn trên đồ thị."""
        for point in path:
            if point not in [self.map_instance.start_point, self.map_instance.end_point] + self.map_instance.stops:
                self.map_instance.matrix[point[1]][point[0]] = 'X'

    def dijkstra_search_between_points(self, start, goal):
        frontier = []
        heapq.heappush(frontier, (0, start))
        came_from = {start: None}
        cost_so_far = {start: 0}
        # Visualization: Initialize open and closed sets for visualizer
        open_set = set([start])
        closed_set = set()
        while frontier:
            current_cost, current = heapq.heappop(frontier)
            closed_set.add(current)

            if current == goal:
                break

            for next in self._neighbors(current):
                new_cost = cost_so_far[current] + self._move_cost(current, next)
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    heapq.heappush(frontier, (new_cost, next))
                    came_from[next] = current
                    open_set.add(next)
                    
            # Visualization: Update visualizer with current open and closed sets
            if self.visualizer:
                self.visualizer.visualize_search_step(list(open_set), list(closed_set), [])
        
        if goal not in came_from:
            return None, float('inf')

        # Trực tiếp tái tạo đường đi từ 'came_from' mà không cần hàm _reconstruct_path
        path = []
        current = goal
        while current != start:
            path.append(current)
            current = came_from[current]
        path.append(start)
        path.reverse()
        if self.visualizer:
            self.visualizer.update_visualization(path)
        return path, cost_so_far[goal]


    def find_and_path_to_nearest_stop(self,current_position, stops_remaining):
            nearest_stop = None
            shortest_path = None
            shortest_path_cost = float('inf')
            for stop in stops_remaining:
                path, cost = self.dijkstra_search_between_points(current_position, stop)
                if cost < shortest_path_cost:
                    nearest_stop = stop
                    shortest_path = path
                    shortest_path_cost = cost
            return nearest_stop, shortest_path, shortest_path_cost
        
    def dijkstra_search(self):
        total_path = []
        total_cost = 0
        current_position = self.map_instance.start_point
        stops = self.map_instance.stops.copy()
        goal = self.map_instance.end_point
        

        while stops:
            nearest_stop, path_to_stop, cost_to_stop = self.find_and_path_to_nearest_stop(current_position,
                                                                                     stops)
            if path_to_stop is None:
                return None, float('inf')  # Path not found to one of the stops
            total_cost += cost_to_stop
            total_path.extend(path_to_stop[:-1])  # Exclude the last point to avoid duplication
            current_position = nearest_stop
            stops.remove(nearest_stop)

        # Add final leg from last stop to end point
        final_path, final_cost = self.dijkstra_search_between_points(current_position, goal)
        if final_path is None:
            return None, float('inf')  # Path not found from last stop to end point
        total_cost += final_cost
        total_path.extend(final_path)

        # Now that the complete path is determined, mark it on the graph
        self._mark_path_on_graph(total_path)
        if self.visualizer:
            self.visualizer.update_visualization(total_path)
        return total_path, total_cost
