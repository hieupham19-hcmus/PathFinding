import random
import numpy as np

class Polygon:
    def __init__(self, points, edge_char='*', direction=(0, 0)):
        self.points = points
        self.polygon_path = []  # [(x, y) for x, y in points]
        self.edge_char = edge_char
        self.direction = direction
        #randomly select a color
        self.color = "#%06x" % random.randint(0, 0xFFFFFF)


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

        # self.polygon_path.append((x1, y1))

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
        return (max(p[0], r[0]) >= q[0] >= min(p[0], r[0]) and
                max(p[1], r[1]) >= q[1] >= min(p[1], r[1]))

    def is_on_edge_or_inside_polygon(self, point):
        """Check if a point is on the edge of or inside the polygon."""
        # Check if the point is on any of the polygon's edges
        for i in range(len(self.points)):
            if self._on_segment(self.points[i], point, self.points[(i + 1) % len(self.points)]):
                return True  # The point is on an edge

        # Ray casting algorithm to check if the point is inside the polygon
        count = 0
        p2 = (np.inf, point[1])
        for i in range(len(self.points)):
            p1 = self.points[i]
            p3 = self.points[(i + 1) % len(self.points)]

            if self._lines_intersect(point, p2, p1, p3):
                if self._orientation(p1, point, p3) == 0:
                    return self._on_segment(p1, point, p3)
                count += 1

        # Point is inside if count is odd
        return count % 2 == 1

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
        # randomly select a direction
        direction = self.direction[0]
        new_points = [(x + direction[0], y + direction[1]) for x, y in self.points]
        if all(0 < x < matrix_size[1] - 1 and 0 < y < matrix_size[0] - 1 for x, y in new_points):
            self.points = new_points
        # Implement edge collision handling as needed

