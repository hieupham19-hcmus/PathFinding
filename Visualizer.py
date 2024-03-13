import pygame
import sys
import random  
from Map import Map

class Visualizer:
    def __init__(self, map_obj, preferred_window_size=(800, 600)):
        pygame.init()
        self.map = map_obj
        self.cell_size = min(preferred_window_size[0] // self.map.width, preferred_window_size[1] // self.map.height)
        # Set the width and height of the screen to exactly fit the map dimensions
        self.width = self.map.width * self.cell_size
        self.height = self.map.height * self.cell_size
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption('Pathfinding Visualizer')
        self.clock = pygame.time.Clock()  # For frame rate limiting
        self.dirty_rects = []
        self.font = pygame.font.SysFont(None, 24)  # Font for text
        self.paused = False  # New attribute to control pause/resume

    def draw_grid(self):
        for y in range(self.map.height):
            for x in range(self.map.width):
                rect = pygame.Rect(x * self.cell_size, (self.map.height - 1 - y) * self.cell_size,  # Invert y-axis
                                   self.cell_size, self.cell_size)
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
        # Draw the coordinates along the bottom
        for x in range(self.map.width):
            text = self.font.render(str(x), True, (0, 0, 0))
            self.screen.blit(text, (x * self.cell_size + 5, self.height - 25))

        # Draw the coordinates along the left side
        for y in range(self.map.height):
            text = self.font.render(str(y), True, (0, 0, 0))
            self.screen.blit(text, (5, (self.map.height - 1 - y) * self.cell_size + 5))

    def update_visualization(self, random_color, path=None):
     

        if path:
            for x, y in path:
                # Calculate rectangle for path cell
                rect = pygame.Rect(x * self.cell_size, (self.map.height - 1 - y) * self.cell_size, self.cell_size, self.cell_size)
                # Draw the path cell
                pygame.draw.rect(self.screen, random_color, rect)
                # Add the rectangle to the dirty_rects list to update just this part
                self.dirty_rects.append(rect)

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


    def visualize_search_step(self, open_set, closed_set, current_path):
        # Fill the screen to clear old visuals
        self.screen.fill((220, 220, 220))

        # Draw the grid and labels
        self.draw_grid()
        #self.draw_labels()

        # Visualize the open set
        for node in open_set:
            self.color_cell(node, (0, 255, 255))  # Cyan for open set

        # Visualize the closed set
        for node in closed_set:
            self.color_cell(node, (255, 165, 0))  # Orange for closed set

        # Visualize the current path
        for node in current_path:
            self.color_cell(node, (255, 255, 0))  # Yellow for current path

        # Update the display and wait to visualize the speed
        pygame.display.flip()
        pygame.time.delay(100)  # Delay in milliseconds

    def color_cell(self, cell, color):
        x, y = cell
        rect = pygame.Rect(x * self.cell_size, (self.map.height - 1 - y) * self.cell_size, self.cell_size,
                           self.cell_size)
        pygame.draw.rect(self.screen, color, rect)

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
