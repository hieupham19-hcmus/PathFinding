import pygame
import sys
import random  
from Map import Map

class Visualizer:
    
    def __init__(self, map_obj, preferred_window_size=(800, 600)):
        pygame.init()
        self.map = map_obj
        self.cell_size = min(preferred_window_size[0] // self.map.width, preferred_window_size[1] // self.map.height)
        self.top_spacing = self.cell_size * 2
        # Set the width and height of the screen to exactly fit the map dimensions
        self.width = self.map.width * self.cell_size
        self.height = self.map.height * self.cell_size + +self.top_spacing
        self.screen = pygame.display.set_mode((self.width, self.height))
        self.screen.fill((255, 255, 255)) 
        pygame.display.set_caption('Pathfinding Visualizer')
        self.clock = pygame.time.Clock()  # For frame rate limiting
        self.dirty_rects = []
        self.font = pygame.font.SysFont('Verdana', 12)  # Font for text
        self.paused = False  # New attribute to control pause/resume

        

    #def draw_header(self):
        #hello_text = self.font.render("Total cost:", True, (255, 0, 255))  
        #text_rect = hello_text.get_rect(center=(self.width // 2, self.top_spacing // 2))  # Position the text at the top center
        #self.screen.blit(hello_text, text_rect)

    def draw_grid(self):
        for y in range(self.map.height):
            for x in range(self.map.width):
                rect = pygame.Rect(x * self.cell_size, self.top_spacing + (self.map.height - 1 - y) * self.cell_size,
                                   self.cell_size, self.cell_size)  # Add top_spacing

                if (x, y) == self.map.start_point:
                    color = (0, 255, 0)  # Start - Green
                elif (x, y) == self.map.end_point:
                    color = (255, 0, 0)  # End - Red
                elif self.map.matrix[y][x] == 'B':
                    color = (0, 0, 255)  # Stop - Blue
                elif self.map.matrix[y][x] == '#':
                    color = (0, 0, 0)  # Obstacle - Black
                else:
                    color = (255, 255, 255)  # Empty cell - White

                for polygon in self.map.polygons:
                    if (x, y) in polygon.polygon_path:
                        color = polygon.color

                pygame.draw.rect(self.screen, color, rect)

                # Draw the grid lines
                pygame.draw.rect(self.screen, (0, 0, 0), rect, 1)  # Black grid lines

    def draw_labels(self):
        # Draw the coordinates along the bottom with spacing
        for x in range(self.map.width):
            text = self.font.render(str(x), True, (0, 0, 0))
            self.screen.blit(text, (x * self.cell_size + 5, self.height - 25 + self.top_spacing))

        # Draw the coordinates along the left side with spacing
        for y in range(self.map.height):
            text = self.font.render(str(y), True, (0, 0, 0))
            self.screen.blit(text, (5, self.top_spacing + (self.map.height - 1 - y) * self.cell_size + 5))

    def update_visualization(self, random_color, path=None, total_cost=None):
        if path:
            for x, y in path:
                # Calculate rectangle for path cell
                rect = pygame.Rect(x * self.cell_size, self.top_spacing + (self.map.height - 1 - y) * self.cell_size,
                                self.cell_size, self.cell_size)
                # Draw the path cell
                pygame.draw.rect(self.screen, random_color, rect)
                # Add the rectangle to the dirty_rects list to update just this part
                self.dirty_rects.append(rect)

        if total_cost is not None:
            # Render the total cost text
            cost_text = self.font.render(f"Total Cost: {total_cost}", True, (255, 0, 255))  # Render text in pink color
            text_rect = cost_text.get_rect(center=(self.width // 2, self.top_spacing // 2))  # Position text above grid
            text_rect.top = self.top_spacing // 2  # Adjust the top position of the text
            self.screen.blit(cost_text, text_rect)

        # Update only the rectangles that have changed in this frame.
        pygame.display.update(self.dirty_rects)

        # Clear the list of dirty rectangles so that only new changes are updated next frame.
        self.dirty_rects.clear()


    def run_visualization(self):
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False  # Only exit loop if QUIT event is triggered
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_SPACE:
                        self.paused = not self.paused

            if not self.paused:
                self.update_visualization()

            pygame.time.delay(10)  # Continue checking for events with a slight delay to reduce CPU usage

        pygame.quit()  # Cleanup and close the window once the loop is exited

    def visualize_after_move_polygon(self, current_position, polygons):
        # Clear screen
        self.screen.fill((255, 255, 255))

        # Redraw the grid with static and dynamic elements
        self.draw_grid()

        # Visualize the current position
        self.update_visualization_with_current_position(current_position)

        # Refresh the display
        pygame.display.flip()

    def update_visualization_with_current_position(self, position):
        # Calculate the rectangle for the current cell
        rect = pygame.Rect(position[0] * self.cell_size,
                           self.top_spacing + (self.map.height - 1 - position[1]) * self.cell_size,
                           self.cell_size, self.cell_size)
        # Draw the current cell with a distinct color, e.g., orange
        pygame.draw.rect(self.screen, (255, 165, 0), rect)  # Orange for the current position
        self.dirty_rects.append(rect)

        # Update the display for only the modified region
        pygame.display.update(self.dirty_rects)
        self.dirty_rects.clear()

    # def visualize_after_move_polygon(self, current_path, polygons):
    #     # Clear screen
    #     self.screen.fill((255, 255, 255))
    #
    #     # Redraw the grid with static and dynamic elements
    #     self.draw_grid()
    #
    #     # Visualize the current path
    #     self.update_visualization_with_path(current_path)
    #
    #     # Refresh the display
    #     pygame.display.flip()
    #
    # def update_visualization_with_path(self, path):
    #     for x, y in path:
    #         rect = pygame.Rect(x * self.cell_size, self.top_spacing + (self.map.height - 1 - y) * self.cell_size,
    #                            self.cell_size, self.cell_size)
    #         pygame.draw.rect(self.screen, (255, 165, 0), rect)  # Orange for the path
    #         self.dirty_rects.append(rect)
    #
    #     # Optionally update labels or other UI elements here
    #     pygame.display.update(self.dirty_rects)
    #     self.dirty_rects.clear()

    def color_cell(self, cell, color, update_only=False):
        x, y = cell
        rect = pygame.Rect(x * self.cell_size, self.top_spacing + (self.map.height - 1 - y) * self.cell_size,
                           self.cell_size, self.cell_size)
        if not update_only or rect not in self.dirty_rects:
            pygame.draw.rect(self.screen, color, rect)
            self.dirty_rects.append(rect)

    def run_event_loop(self):
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_SPACE:
                        self.paused = not self.paused

            if not self.paused:
                # Do pathfinding steps and visualization updates here
                # Remember to add modified regions to self.dirty_rects
                pass

            self.clock.tick(60)  # Limit to 60 FPS
            pygame.display.flip()
#
        pygame.quit()  # Cleanup and close the window once the loop is exited
        sys.exit()
