import numpy as np
import heapq

class Polygon:
    def __init__(self, points, edge_char='*', direction=(0, 0)):
        self.points = points
        self.polygon_path = [] #[(x, y) for x, y in points]
        self.edge_char = edge_char
        self.direction = [(1, 0), (0, 1), (-1, 0), (0, -1)]

    def _draw_line(self, matrix, p1, p2):
        # Bresenham's line algorithm
        x0, y0 = p1
        x1, y1 = p2
        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx + dy  # Note the change here
        
        while True:
            matrix[y0][x0] = self.edge_char  # Changed to 2D list indexing
            self.polygon_path.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x0 += sx
            if e2 <= dx:
                err += dx
                y0 += sy
        
        #self.polygon_path.append((x1, y1))
                
    def draw(self, matrix):  # Renamed for clarity
        for i in range(len(self.points)):
            p1 = self.points[i]
            p2 = self.points[(i + 1) % len(self.points)]
            self._draw_line(matrix, p1, p2)

    def intersects_edge(self, p1, p2):
        """Check if the line segment from p1 to p2 intersects any edge of the polygon."""
        for i in range(len(self.points)):
            p3 = self.points[i]
            p4 = self.points[(i + 1) % len(self.points)]

            if self._lines_intersect(p1, p2, p3, p4):
                return True
        return False
    
   
    def is_on_edge_or_inside_polygon(self, point):
        if point in self.polygon_path:
            return True
            
        # Check if the point is on the edge of the polygon
        for i in range(len(self.points)):
            p1 = self.points[i]
            p2 = self.points[(i + 1) % len(self.points)]
            
            if self._orientation(p1, p2, point) == 0 and self._on_segment(p1, point, p2):
                return True  # The point is on the edge of the polygon

        # Ray casting algorithm to check if the point is inside the polygon
        n = len(self.points)
        inside = False
        p2x, p2y = 0, 0
        xints = 0
        p1x, p1y = self.points[0]
        for i in range(n + 1):
            p2x, p2y = self.points[i % n]
            if point[1] > min(p1y, p2y):
                if point[1] <= max(p1y, p2y):
                    if point[0] <= max(p1x, p2x):
                        if p1y != p2y:
                            xints = (point[1] - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or point[0] <= xints:
                            inside = not inside
            p1x, p1y = p2x, p2y

        return inside



    def _orientation(self, p, q, r):
        """Determine the orientation of the triplet (p, q, r).
        Returns:
        0 --> p, q, and r are collinear
        1 --> Clockwise
        2 --> Counterclockwise
        """
        val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
        if val == 0:
            return 0
        return 1 if val > 0 else 2

    def _on_segment(self, p, q, r):
        """Given three collinear points p, q, and r, check if point q lies on line segment 'pr'."""
        return (q[0] <= max(p[0], r[0]) and q[0] >= min(p[0], r[0]) and
                q[1] <= max(p[1], r[1]) and q[1] >= min(p[1], r[1]))

    def _lines_intersect(self, p1, p2, p3, p4):
        """Check if the line segments p1p2 and p3p4 intersect."""
        o1 = self._orientation(p1, p2, p3)
        o2 = self._orientation(p1, p2, p4)
        o3 = self._orientation(p3, p4, p1)
        o4 = self._orientation(p3, p4, p2)

        # General case
        if o1 != o2 and o3 != o4:
            return True

        # Special Cases
        # p1, p2, and p3 are collinear and p3 lies on segment p1p2
        if o1 == 0 and self._on_segment(p1, p3, p2):
            return True

        # p1, p2, and p4 are collinear and p4 lies on segment p1p2
        if o2 == 0 and self._on_segment(p1, p4, p2):
            return True

        # p3, p4, and p1 are collinear and p1 lies on segment p3p4
        if o3 == 0 and self._on_segment(p3, p1, p4):
            return True

        # p3, p4, and p2 are collinear and p2 lies on segment p3p4
        if o4 == 0 and self._on_segment(p3, p2, p4):
            return True

        # Doesn't fall in any of the above cases
        return False
    
    def move(self, matrix_size):
        #randomly select a direction
        direction = self.direction[np.random.randint(0, 4)]
        new_points = [(x + direction[0], y + direction[1]) for x, y in self.points]
        if all(0 < x < matrix_size[1] - 1 and 0 < y < matrix_size[0] - 1 for x, y in new_points):
            self.points = new_points
        # Implement edge collision handling as needed
        
        
        

class Map:
    def __init__(self):
        self.height = 0
        self.width = 0
        self.polygons = []
        self.stops = []
        self.matrix = []
        self.start_point = (0, 0)
        self.end_point = (0, 0)

    def get_input_from_file(self, file_name) -> True:
        with open(file_name, 'r') as file:
            lines = file.read().splitlines()

        self.width, self.height = map(int, lines[0].split(','))
        self.width += 1
        self.height += 1

        self.matrix = [['0' for _ in range(self.width)] for _ in range(self.height)]
        for i in range(self.width):
            self.matrix[0][i] = self.matrix[-1][i] = '#'
        for i in range(self.height):
            self.matrix[i][0] = self.matrix[i][-1] = '#'

        points = list(map(int, lines[1].split(',')))
        self.start_point = (points[0], points[1])
        self.end_point = (points[2], points[3])
        
        for i in range(4, len(points), 2):
            stop = (points[i], points[i+1])
            self.stops.append(stop)
            self.matrix[stop[1]][stop[0]] = 'B'

        number_of_polygons = int(lines[2])
        
        for line in lines[3: 3 + number_of_polygons]:
            raw_polygon = list(map(int, line.split(',')))
            # Ensure raw_polygon has an even number of elements
            if len(raw_polygon) % 2 == 0:
                polygon_points = [(raw_polygon[i], raw_polygon[i + 1]) for i in range(0, len(raw_polygon), 2)]
                self.add_polygon(polygon_points)
            else:
                raise ValueError("Polygon data is not correctly formatted (odd number of points).")
                #print("Error: ")
                #return False

        for polygon in self.polygons:
            if(polygon.is_on_edge_or_inside_polygon(self.start_point) or polygon.is_on_edge_or_inside_polygon(self.end_point)):\
                raise ValueError("Error: Start and end points are not outside of polygons.")
                #print("")
                #return False
            else:
                self.matrix[self.start_point[1]][self.start_point[0]] = 'S'
                self.matrix[self.end_point[1]][self.end_point[0]] = 'G'
        
        return True
    
    def add_polygon(self, points, edge_char='*'):
        # Create a temporary polygon object for intersection checks
        temp_polygon = Polygon(points, edge_char)

        # Check for edge intersections with existing polygons
        for existing_polygon in self.polygons:
            for i, point in enumerate(temp_polygon.points):
                next_point = temp_polygon.points[(i + 1) % len(temp_polygon.points)]
                if existing_polygon.intersects_edge(point, next_point):
                    raise ValueError("Polygon edges intersect with an existing polygon.")

        # Check if any point of the new polygon is inside an existing polygon
        for point in temp_polygon.points:
            for existing_polygon in self.polygons:
                if existing_polygon.is_on_edge_or_inside_polygon(point):
                    raise ValueError("A point of the new polygon is inside an existing polygon.")

        # If no intersections or inside points, add the polygon to the map
        self.polygons.append(temp_polygon)
        temp_polygon.draw(self.matrix)


    def print_map(self):
        for row in self.matrix[::-1]:
            print(''.join(row))

    
    def _heuristic(self, a, b):
        return np.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


    def _neighbors(self, point):
        dirs = [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (1, -1), (-1, -1), (-1, 1)]  # Including diagonals
        result = []
        for d in dirs:
            next_point = (point[0] + d[0], point[1] + d[1])
            if 0 <= next_point[0] < self.width and 0 <= next_point[1] < self.height and self.matrix[next_point[1]][next_point[0]] != '#':
                # Check if the movement intersects any polygon
                intersects_polygon = False
                for polygon in self.polygons:
                    if polygon.intersects_edge(point, next_point):  # Assumes implementation of this method in Polygon
                        intersects_polygon = True
                        break
                if not intersects_polygon:
                    result.append(next_point)
        return result


    def a_star_search(self):
        """Find the path from start_point to end_point using A* search, including diagonals."""
        start, goal = self.start_point, self.end_point
        frontier = []
        heapq.heappush(frontier, (0, start))
        came_from = {start: None}
        cost_so_far = {start: 0}

        while frontier:
            current = heapq.heappop(frontier)[1]

            if current == goal:
                break

            for next in self._neighbors(current):
                if current[0] != next[0] and current[1] != next[1]:  # Diagonal move
                    new_cost = cost_so_far[current] + 1.5  
                else:  # Straight move
                    new_cost = cost_so_far[current] + 1
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + self._heuristic(next, goal)
                    heapq.heappush(frontier, (priority, next))
                    came_from[next] = current

        if goal not in came_from:
            return None, float('inf')  

        path = self._reconstruct_path(came_from, start, goal)
        return path, cost_so_far.get(goal, float('inf'))


    def _reconstruct_path(self, came_from, start, goal):
        """Reconstruct the path from start to goal."""
        current = goal
        path = []
        while current != start:
            if self.matrix[current[1]][current[0]] != 'G':
                self.matrix[current[1]][current[0]] = 'X'  # Mark the path on the map
            path.append(current)
            current = came_from[current]
        path.append(start)  # optional
        path.reverse()  # optional
        return path
    
    def greedy_best_first_search(self):
        """Find the path from start_point to end_point using Greedy Best-First Search,
        including the cost of straight and diagonal moves, and keep track of the path cost."""
        start, goal = self.start_point, self.end_point
        frontier = []
        heapq.heappush(frontier, (0, start))  # Priority is just the heuristic
        came_from = {start: None}
        cost_so_far = {start: 0}  # Cost from start to the current node

        while frontier:
            current = heapq.heappop(frontier)[1]

            if current == goal:
                break

            for next in self._neighbors(current):
                if current[0] != next[0] and current[1] != next[1]:  # Diagonal move
                    new_cost = cost_so_far[current] + 1.5
                else:  # Straight move
                    new_cost = cost_so_far[current] + 1

                if next not in came_from or new_cost < cost_so_far.get(next, float('inf')):  # Check for better path
                    cost_so_far[next] = new_cost  # Update cost
                    priority = self._heuristic(next, goal)
                    heapq.heappush(frontier, (priority, next))
                    came_from[next] = current

        if goal not in came_from:
            return None, float('inf')  

        path = self._reconstruct_path(came_from, start, goal)
        return path, cost_so_far.get(goal, float('inf'))

    def breadth_first_search(self):
        start, goal = self.start_point, self.end_point
        frontier = [start]  # Use a list as a queue for BFS
        came_from = {start: None}
        cost_so_far = {start: 0}  # Track the cost to reach each node

        while frontier:
            current = frontier.pop(0)  # FIFO queue

            if current == goal:
                break

            for next in self._neighbors(current):
                # Determine if the move is diagonal or straight
                if current[0] != next[0] and current[1] != next[1]:  # Diagonal move
                    new_cost = cost_so_far[current] + 1.5
                else:  # Straight move
                    new_cost = cost_so_far[current] + 1

                if next not in came_from or new_cost < cost_so_far.get(next, float('inf')):
                    frontier.append(next)
                    came_from[next] = current
                    cost_so_far[next] = new_cost  # Update cost with diagonal consideration

        # Check if the goal was reached
        if goal not in came_from:
            # No path found, handle accordingly
            return None, float('inf')  # Example: return None for the path and infinity for the cost

        # Path was found, reconstruct and return it
        path = self._reconstruct_path(came_from, start, goal)
        return path, cost_so_far.get(goal, float('inf'))  # Return the path and its cost

    def depth_first_search(self):
        start, goal = self.start_point, self.end_point
        frontier = [start]  # Use a list as a stack for DFS
        came_from = {start: None}
        cost_so_far = {start: 0}  # Track the cost to reach each node

        while frontier:
            current = frontier.pop()  # LIFO stack

            if current == goal:
                break

            for next in self._neighbors(current):
                # Determine if the move is diagonal or straight
                if current[0] != next[0] and current[1] != next[1]:  # Diagonal move
                    new_cost = cost_so_far[current] + 1.5
                else:  # Straight move
                    new_cost = cost_so_far[current] + 1

                if next not in came_from or new_cost < cost_so_far.get(next, float('inf')):
                    frontier.append(next)
                    came_from[next] = current
                    cost_so_far[next] = new_cost  # Update cost with diagonal consideration

        if goal in came_from:
            return self._reconstruct_path(came_from, start, goal), cost_so_far[goal]  # Return the path and total cost
        else:
            return None, float('inf')  # No path found
        
    def move_polygons(self):
        directions = [(1, 0), (0, 1), (-1, 0), (0, -1)]  # Four possible directions (right, up, left, down)
        for polygon in self.polygons:
            # Try to move the polygon in a random direction
            valid_move_found = False
            np.random.shuffle(directions)  # Shuffle directions to try them in random order
            for direction in directions:
                new_points = [(x + direction[0], y + direction[1]) for x, y in polygon.points]
                # Check if new position is within bounds and does not collide with other polygons
                if self._is_valid_move(new_points) and not self._collides_with_other_polygons(new_points, polygon):
                    polygon.points = new_points
                    valid_move_found = True
                    break
            if not valid_move_found:
                # If no valid move is found, the polygon does not move
                # Alternatively, implement a more sophisticated strategy here
                pass

            # Redraw the map with updated polygon positions
            self._redraw_map()

    def _is_valid_move(self, points):
        """Check if the new position of a polygon is within the map boundaries."""
        return all(0 < x < self.width - 1 and 0 < y < self.height - 1 for x, y in points)

    def _collides_with_other_polygons(self, new_points, moving_polygon):
        """Check if moving a polygon to new points would result in a collision with any other polygon."""
        for polygon in self.polygons:
            if polygon == moving_polygon:
                continue  # Skip the moving polygon itself
            for point in new_points:
                if polygon.is_on_edge_or_inside_polygon(point):
                    return True  # Collision detected
        return False

    def _redraw_map(self):
        """Clear and redraw the map based on the current positions of polygons and other entities."""
        # Clear non-fixed elements from the map
        for i in range(self.height):
            for j in range(self.width):
                if self.matrix[i][j] not in ('S', 'G', '#', 'B'):  # Preserve start, goal, walls, and stops
                    self.matrix[i][j] = '0'
        # Redraw all polygons
        for polygon in self.polygons:
            polygon.polygon_path = []
            polygon.draw(self.matrix)
            
