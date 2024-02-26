import numpy as np


class Map:
    def __init__(self):
        self.height = 0
        self.width = 0
        self.amount_of_polygon = 0
        self.amount_of_stop = 0
        self.stops = []
        self.polygons = []
        self.matrix = []
        self.start_point = (0, 0)
        self.end_point = (0, 0)
    
    
    # class Polygon:
        
     
    def get_input_from_file(self, file_name : str):
        with open(file_name, 'r') as file:
            lines = file.read().splitlines()
            
        self.width, self.height = map(int, lines[0].split(','))
        self.width += 1
        self.height += 1
        
        self.matrix = np.full((self.height, self.width), '0', dtype = str)
            
        self.matrix[0, :] = self.matrix[-1, :] = '#'  
        self.matrix[:, 0] = self.matrix[:, -1] = '#' 
    
        points = list(map(int, lines[1].split(',')))
        self.start_point = (points[0], points[1])
        self.end_point = (points[2], points[3])
        self.matrix[self.start_point[::-1]] = 'S'
        self.matrix[self.end_point[::-1]] = 'G'
        
        stop_points = points[4:]
        for i in range(0, len(stop_points), 2):
            stop = tuple(stop_points[i:i+2][::-1])  # Correcting for row-column indexing
            self.stops.append(stop)
            self.matrix[stop] = 'B'
            self.amount_of_stop += 1
        
        # Polygons
        if len(lines) > 2:
            self.amount_of_polygon = int(lines[2])
            for line in lines[3: 3 + self.amount_of_polygon]:
                raw_polygon = list(map(int, line.split(',')))
                polygon = [(raw_polygon[i], raw_polygon[i + 1]) for i in range(0, len(raw_polygon), 2)]
                self.polygons.append(polygon)
                self._draw_polygon(polygon)
    
    def _draw_polygon(self, polygon : list) -> list:
        def _draw_line(p1 : tuple, p2 : tuple) -> list:   
            x0, y0 = p1
            x1, y1 = p2
            dx = abs(x1 - x0)
            dy = abs(y1 - y0)
            sx = 1 if x0 < x1 else -1
            sy = 1 if y0 < y1 else -1
            err = dx - dy
            while True:
                self.matrix[y0, x0] = '*'
                if x0 == x1 and y0 == y1:
                    break
                e2 = 2 * err
                if e2 > -dy:
                    err -= dy
                    x0 += sx
                if e2 < dx:
                    err += dx
                    y0 += sy
        """
        def _flood_fill(self, x, y, oldChar, newChar):
            if x < 0 or x >= self.width or y < 0 or y >= self.height:
                return
            if self.matrix[y, x] != oldChar:
                return
            self.matrix[y, x] = newChar
            self._flood_fill(x + 1, y, oldChar, newChar)
            self._flood_fill(x - 1, y, oldChar, newChar)
            self._flood_fill(x, y + 1, oldChar, newChar)
            self._flood_fill(x, y - 1, oldChar, newChar)
        """

        for i in range(len(polygon)):
            p1 = polygon[i]
            p2 = polygon[(i + 1) % len(polygon)] 
            _draw_line(p1, p2)
        
    # fill the area inside the polygon with "*"
   
    
    # Assuming _is_valid_position is implemented later if needed
    def _is_valid_position(self, x, y):
        #TODO: check if the position is valid (not in the polygon or border of the map)
        pass 
    
    def print_map(self):
        for row in self.matrix[::-1]:  # Print from top to bottom
            print(''.join(row))
