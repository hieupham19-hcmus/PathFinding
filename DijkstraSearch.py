import heapq
import random

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
      
        while frontier:
            current_cost, current = heapq.heappop(frontier)

            if current == goal:
                break

            for next in self._neighbors(current):
                new_cost = cost_so_far[current] + self._move_cost(current, next)
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    heapq.heappush(frontier, (new_cost, next))
                    came_from[next] = current
                    

        
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
  
        return path, cost_so_far[goal]

    def find_best_path(self,current_start, stops, goal, path_to_goal=[], total_cost=0, best_result=[float('inf'), []]):

        if not stops:
            # Khi không còn điểm dừng, tính toán chi phí đến điểm kết thúc và cập nhật kết quả tốt nhất nếu cần
            final_path, final_cost = self.dijkstra_search_between_points(current_start, goal)
            total_cost += final_cost
            if total_cost < best_result[0]:
                best_result[0] = total_cost
                best_result[1] = path_to_goal + final_path
        else:
            for i, stop in enumerate(stops):
                next_stops = stops[:i] + stops[i+1:]
                path, cost = self.dijkstra_search_between_points(current_start, stop)
                if total_cost + cost >= best_result[0]:
                    continue
                # Gọi đệ quy với điểm dừng tiếp theo và cập nhật đường đi và tổng chi phí
                self.find_best_path(stop, next_stops, goal, path_to_goal + path, total_cost + cost, best_result)

    
        
    def dijkstra_search(self):
        self.visualizer.draw_grid()
        total_path = []
        total_cost = 0
        current_position = self.map_instance.start_point
        stops = self.map_instance.stops.copy()
        goal = self.map_instance.end_point
        

        

        best_result = [float('inf'), []]  # This sets up best_result with an initial infinite cost and an empty path.
        self.find_best_path(current_position,stops,goal,total_path,total_cost,best_result=best_result)

        # path_to_goal.extend(final_path)
        if self.visualizer:
            random_color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
            self.visualizer.update_visualization(random_color,best_result[1], best_result[0])
      
        return best_result[1], best_result[0]