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
    
    def add_polygon(self, points, edge_char='#', direction=(0, 0)):
        self.polygons.append(self.Polygon(points, edge_char, direction))
        self._draw_polygon(points)
    
    class Polygon:
        def __init__(self, points, edge_char='#', direction=(0, 0)):
            self.points = points
            self.edge_char = edge_char
            self.direction = direction  # Vector di chuyển mới
        
        def _draw_polygon(self, polygon : list) -> list:
            def _draw_line(p1: tuple, p2: tuple):
                self.polygon_path.append(p1)
                self.polygon_path.append(p2)
                x0, y0 = p1
                x1, y1 = p2
                dx = abs(x1 - x0)
                dy = abs(y1 - y0)
                sx = 1 if x0 < x1 else -1
                sy = 1 if y0 < y1 else -1
                err = dx - dy
                while True:
                    self.matrix[y0, x0] = '*'
                    self.polygon_path.append((x0, y0))
                    if x0 == x1 and y0 == y1:
                        break
                    e2 = 2 * err
                    if e2 > -dy:
                        err -= dy
                        x0 += sx
                    if e2 < dx:
                        err += dx
                        y0 += sy

            def _is_inside_polygon_or_on_the_edge(self, x, y) -> bool:
                # Kiểm tra xem điểm có nằm trên cạnh của đa giác
                for i in range(len(self.points)):
                    p1 = self.points[i]
                    p2 = self.points[(i + 1) % len(self.points)]
                    dx = p2[0] - p1[0]
                    dy = p2[1] - p1[1]
                    if dx == 0 and x == p1[0]:
                        if min(p1[1], p2[1]) <= y <= max(p1[1], p2[1]):
                            return True
                    elif dy == 0 and y == p1[1]:
                        if min(p1[0], p2[0]) <= x <= max(p1[0], p2[0]):
                            return True
                    elif dx != 0 and dy != 0:
                        m = dy / dx
                        b = p1[1] - m * p1[0]
                        if abs(y - (m * x + b)) < 1e-5:
                            if min(p1[0], p2[0]) <= x <= max(p1[0], p2[0]) and min(p1[1], p2[1]) <= y <= max(p1[1], p2[1]):
                                return True

                # Kiểm tra xem điểm có nằm bên trong đa giác sử dụng phương pháp ray casting
                inside = False
                n = len(self.points)
                p1x, p1y = self.points[0]
                for i in range(n + 1):
                    p2x, p2y = self.points[i % n]
                    if y > min(p1y, p2y):
                        if y <= max(p1y, p2y):
                            if x <= max(p1x, p2x):
                                if p1y != p2y:
                                    xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                                if p1x == p2x or x <= xinters:
                                    inside = not inside
                    p1x, p1y = p2x, p2y

                return inside
                
                
            
            def _flood_fill(self, x, y, fill_char="*"):
                # Kiểm tra điều kiện cơ bản cho đệ quy
                if not self._is_inside_polygon_or_on_the_edge(x, y) or self.matrix[y][x] not in ['0', 'S', 'G']:
                    return
                # Đánh dấu điểm hiện tại
                self.matrix[y][x] = fill_char
                # Gọi đệ quy cho các ô lân cận
                self._flood_fill(x + 1, y, fill_char)
                self._flood_fill(x - 1, y, fill_char)
                self._flood_fill(x, y + 1, fill_char)
                self._flood_fill(x, y - 1, fill_char)
            
            for i in range(len(polygon)):
                p1 = polygon[i]
                p2 = polygon[(i + 1) % len(polygon)]
                self.polygon
                _draw_line(p1, p2)

        def move(self, matrix_size : tuple):
            """
            Cập nhật vị trí của đa giác dựa trên hướng di chuyển và kiểm tra viền.
            :param matrix_size: Kích thước của ma trận (height, width).
            """
            new_points = [(x + self.direction[0], y + self.direction[1]) for x, y in self.points]
            # Kiểm tra xem sau khi di chuyển có điểm nào vượt qua giới hạn ma trận không
            if all(0 <= x < matrix_size[1] and 0 <= y < matrix_size[0] for x, y in new_points):
                self.points = new_points
            else:
                # Xử lý khi chạm viền tại đây, ví dụ: dừng lại hoặc bật lại
                pass
        
        def redraw(self):
            """
            Vẽ lại đa giác lên ma trận và xóa ma trận cũ
            """
            self._draw_polygon(self.points)

        
    # Assuming _is_valid_position is implemented later if needed
    def _is_valid_position(self, x, y):
        #TODO: check if the position is valid (not in the polygon or border of the map)
        pass 
    
    def print_map(self):
        for row in self.matrix[::-1]:  # Print from top to bottom
            print(''.join(row))
