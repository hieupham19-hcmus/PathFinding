# Pathfinding Robot Project

## Introduction

This project is a part of the Artificial Intelligence curriculum at the University of Science, HCMC, guided by Dr. Le Nguyen Nhut Truong. It aims to explore and implement pathfinding algorithms to navigate a robot from a starting point to a destination while avoiding obstacles on a 2D map. The essence of this project lies in understanding the challenges of spatial navigation, algorithmic efficiency, and the application of artificial intelligence principles in solving real-world problems.

Utilizing Python and allowing for the use of graphical libraries, the project challenges students to develop a solution that not only finds a path but also optimizes it according to various constraints and conditions provided. This endeavor is not just about coding; it's about creating a bridge between theoretical algorithms and their practical applications in robotics and AI.

## Input Format

To run the project successfully, input data must be structured as follows:

- **First Line:** Map dimensions - Specifies the size of the map with width, height.

- **Second Line:** Starting and destination points - Defines the coordinates of the starting point (S) and the destination point (G). If the project level involves waypoints, their coordinates are listed here as well.

- **Third Line:** Number of obstacles - Indicates how many polygonal obstacles are present on the map.

- **Subsequent Lines:** Polygonal Obstacles - Each line represents a single obstacle defined by a sequence of coordinates forming a convex polygon. The vertices of the polygon are listed in a clockwise order, with the understanding that the last vertex connects back to the first one.

## Example Input File
22,18
2,2 19,16 2,15 10,5
3
4,4 5,9 8,10 9,5
8,12 8,17 13,12
11,1 11,6 14,6 14,1

### Description

This file provides an example input for the Pathfinding Robot Project. The format of the file follows the input structure required by the project:

- **Map dimensions:** 22 (width) x 18 (height)
- **Starting and destination points:** The coordinates are given as 2,2 (starting point), 19,16 (destination point), with additional waypoints at 2,15 and 10,5.
- **Number of obstacles:** 3
- **Polygonal Obstacles:** Three sets of coordinates are provided, each set representing a single obstacle defined by its vertices. These vertices form convex polygons as described in the project's input format specifications.

