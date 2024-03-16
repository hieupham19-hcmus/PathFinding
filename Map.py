import numpy as np
from Polygon import Polygon


class Map:
    def __init__(self):
        self.height = 0
        self.width = 0
        self.polygons = []
        self.stops = []
        self.matrix = []
        self.start_point = (0, 0)
        self.end_point = (0, 0)
        self.search_steps = 0

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
            stop = (points[i], points[i + 1])
            self.stops.append(stop)
            self.matrix[stop[1]][stop[0]] = 'B'

        number_of_polygons = int(lines[2])

        for line in lines[3: 3 + number_of_polygons]:
            raw_polygon = list(map(int, line.split(',')))
            # Ensure raw_polygon has an even number of elements
            if len(raw_polygon) % 2 == 0:
                polygon_points = [(raw_polygon[i], raw_polygon[i + 1]) for i in range(0, len(raw_polygon), 2)]
                self._add_polygon(polygon_points)
            else:
                raise ValueError("Polygon data is not correctly formatted (odd number of points).")
                # print("Error: ")
                # return False

        for polygon in self.polygons:
            if (polygon._is_on_edge_or_inside_polygon(self.start_point) or
                    polygon._is_on_edge_or_inside_polygon(self.end_point)):
                raise ValueError("Error: Start and end points are not outside of polygons.")
                # print("")
                # return False
            else:
                self.matrix[self.start_point[1]][self.start_point[0]] = 'S'
                self.matrix[self.end_point[1]][self.end_point[0]] = 'G'

        return True

    def _add_polygon(self, points, edge_char='*'):
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
                if existing_polygon._is_on_edge_or_inside_polygon(point):
                    raise ValueError("A point of the new polygon is inside an existing polygon.")

        # If no intersections or inside points, add the polygon to the map
        self.polygons.append(temp_polygon)
        temp_polygon.draw(self.matrix)

    def print_map(self):
        for row in self.matrix[::-1]:
            print(''.join(row))

    # index is the index of the robot that is moving
    def move_polygons(self, index):
        directions = [(1, 0), (0, 1), (-1, 0), (0, -1)]  # Four possible directions (right, up, left, down)
        for polygon in self.polygons:
            valid_move_found = False
            np.random.shuffle(directions)  # Shuffle directions to try them in random order
            original_points = polygon.points[:]
            for direction in directions:
                new_points = [(x + direction[0], y + direction[1]) for x, y in original_points]
                # Check if the move is valid and does not result in collision or enclosing critical points
                if (self._is_valid_move(new_points) and
                    not self._collides_with_other_polygons(new_points, polygon) and
                    not self._does_not_enclose_robot(new_points, index)):
                    temp_polygon = Polygon(new_points)
                    if self._critical_points_not_enclosed(temp_polygon):
                        polygon.points = new_points
                        valid_move_found = True
                        break
            if not valid_move_found:
                # If no valid move is found, the polygon does not move
                pass

        self._redraw_map()

    def _does_not_enclose_robot(self, new_points, robot_position):
        # Temporarily create a new polygon with the new points to use for checking
        temp_polygon = Polygon(new_points)

        # Check if the robot's position is inside or on the edge of the new polygon
        if temp_polygon._is_on_edge_or_inside_polygon(robot_position):
            return False  # The robot is enclosed by the new polygon

        # Ensure robot is not enclosed by any existing polygon (excluding the moving one)
        for polygon in self.polygons:
            if polygon._is_on_edge_or_inside_polygon(robot_position) and polygon.points != new_points:
                return False  # The robot is enclosed by another polygon

        return True  # No polygons enclose the robot

    def _is_valid_move(self, points):
        """Check if the new position of a polygon is within the map boundaries."""
        return all(0 < x < self.width - 1 and 0 < y < self.height - 1 for x, y in points)

    def _collides_with_other_polygons(self, new_points, moving_polygon):
        """Check if moving a polygon to new points would result in a collision with any other polygon."""
        for polygon in self.polygons:
            if polygon == moving_polygon:
                continue  # Skip the moving polygon itself
            for point in new_points:
                if polygon._is_on_edge_or_inside_polygon(point):
                    return True  # Collision detected
        return False

    def _critical_points_not_enclosed(self, new_polygon):
        """Check that start_point, end_point, and stops are not inside the new position of the moving polygon or any other polygon."""
        critical_points = [self.start_point, self.end_point] + self.stops
        for point in critical_points:
            if new_polygon._is_on_edge_or_inside_polygon(point):
                return False  # A critical point is inside the new polygon
            for polygon in self.polygons:
                if polygon._is_on_edge_or_inside_polygon(point):
                    return False  # A critical point is inside another polygon
        return True

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
