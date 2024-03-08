import numpy as np
import heapq

class Polygon:
    def __init__(self, points, edge_char='#', direction=(0, 0)):
        self.points = points
        self.edge_char = edge_char
        self.direction = direction

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
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x0 += sx
            if e2 <= dx:
                err += dx
                y0 += sy
    
    #Ray casting algorithm

    def draw(self, matrix):  # Renamed for clarity
        for i in range(len(self.points)):
            p1 = self.points[i]
            p2 = self.points[(i + 1) % len(self.points)]
            self._draw_line(matrix, p1, p2)

    def _is_inside_polygon(self, point):
        #TODO: Implement the ray casting algorithm to determine if a point is inside the polygon
        pass
    
    def move(self, matrix_size):
        new_points = [(x + self.direction[0], y + self.direction[1]) for x, y in self.points]
        if all(0 <= x < matrix_size[1] and 0 <= y < matrix_size[0] for x, y in new_points):
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

    def get_input_from_file(self, file_name):
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
        self.matrix[self.start_point[1]][self.start_point[0]] = 'S'
        self.matrix[self.end_point[1]][self.end_point[0]] = 'G'

        for i in range(4, len(points), 2):
            stop = (points[i], points[i+1])
            self.stops.append(stop)
            self.matrix[stop[1]][stop[0]] = 'B'

        for line in lines[2:]:
            raw_polygon = list(map(int, line.split(',')))
            # Ensure raw_polygon has an even number of elements
            if len(raw_polygon) % 2 == 0:
                polygon_points = [(raw_polygon[i], raw_polygon[i + 1]) for i in range(0, len(raw_polygon), 2)]
                self.add_polygon(polygon_points)
            else:
                print("Error: Polygon data is not correctly formatted (odd number of points).")

    def add_polygon(self, points, edge_char='#'):
        polygon = Polygon(points, edge_char)
        self.polygons.append(polygon)
        polygon.draw(self.matrix)


    
    def print_map(self):
        for row in self.matrix[::-1]:
            print(''.join(row))

    
    def heuristic(self, a, b):
        """Calculate the Manhattan distance between two points."""
        return max(abs(a[0] - b[0]), abs(a[1] - b[1]))

    def neighbors(self, point):
        """Return a list of walkable neighboring points, including diagonals."""
        dirs = [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (1, -1), (-1, -1), (-1, 1)]  # Including diagonals
        result = []
        for d in dirs:
            next_point = (point[0] + d[0], point[1] + d[1])
            if 0 <= next_point[0] < self.width and 0 <= next_point[1] < self.height and self.matrix[next_point[1]][next_point[0]] != '#':
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

            for next in self.neighbors(current):
                new_cost = cost_so_far[current] + 1  # Same cost for all moves, including diagonals

                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + self.heuristic(next, goal)
                    heapq.heappush(frontier, (priority, next))
                    came_from[next] = current

        return self.reconstruct_path(came_from, start, goal)

    def reconstruct_path(self, came_from, start, goal):
        """Reconstruct the path from start to goal."""
        current = goal
        path = []
        while current != start:
            path.append(current)
            current = came_from[current]
        path.append(start)  # optional
        path.reverse()  # optional
        return path
