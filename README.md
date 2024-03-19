# Pathfinding Robot Project

## Introduction

This project is a part of the Artificial Intelligence curriculum at the University of Science, HCMC, guided by Dr. Le Nguyen Nhut Truong. It aims to explore and implement pathfinding algorithms to navigate a robot from a starting point to a destination while avoiding obstacles on a 2D map. The essence of this project lies in understanding the challenges of spatial navigation, algorithmic efficiency, and the application of artificial intelligence principles in solving real-world problems.

Utilizing Python and allowing for graphical libraries, the project challenges students to develop a solution that finds a path and optimizes it according to various constraints and conditions. This endeavor is not just about coding; it's about creating a bridge between theoretical algorithms and their practical applications in robotics and AI.

## Input Format

To run the project successfully, input data must be structured as follows:

- **First Line:** Map dimensions - Specifies the size of the map with width and height.

- **Second Line:** Starting and destination points - Defines the coordinates of the starting point (S) and the destination point (G). If the project level involves waypoints, their coordinates are also listed here.

- **Third Line:** Number of obstacles - Indicates how many polygonal obstacles are on the map.

Subsequent Lines:** Polygonal Obstacles—Each line represents a single obstacle defined by a sequence of coordinates forming a convex polygon. The polygon's vertices are listed in clockwise order, with the understanding that the last vertex connects back to the first one.

## Example Input File
22,18 \
2,2,19,16,2,15,10,5 \
3 \
4,4,5,9,8,10,9,5 \
8,12,8,17,13,12 \
11,1,11,6,14,6,14,1 
### Description

This file provides an example input for the Pathfinding Robot Project. The format of the file follows the input structure required by the project:

- **Map dimensions:** 22 (width) x 18 (height)
- **Starting and destination points:** The coordinates are 2,2 (starting point), 19,16 (destination point), and additional waypoints at 2,15 and 10,5.
- **Number of obstacles:** 3
- **Polygonal Obstacles:** Three sets of coordinates are provided, each representing a single obstacle defined by its vertices. These vertices form convex polygons as described in the project's input format specifications.

## Pathfinding Algorithm Usage Guide (use release version)

This guide provides instructions on how to run a pathfinding algorithm on a specified map using the `pathfinding.exe` command-line tool. Information on setting up the required environment using `requirements.txt` is also included.

### Running the Pathfinding Tool

To use the pathfinding tool, you'll need to specify the input file and the search algorithm you wish to use. The tool is executed from the command line as follows: \

*Tabspace* ./pathfinding.exe --input <input_file> --search <search_algorithm>

### Search Types

The tool supports the following search algorithms:

- `astar`: A* Search Algorithm
- `greedy`: Greedy Best First Search Algorithm
- `dijkstra`: Dijkstra's Algorithm
- `dynamic`: A Dynamic version of the A* Search Algorithm

## Pathfinding Algorithm Usage Guide (use Source code)
### Setting Up the Environment

Before running the pathfinding tool, ensure your Python environment is set up correctly. This involves installing the necessary dependencies listed in `requirements.txt`

To install these dependencies, use the following command in your terminal:

*Tabspace* pip install -r requirements.txt

Ensure you have `pip` installed and you are in the directory containing the `requirements.txt` file or specify the path to it.

This setup will ensure that all necessary Python packages are installed, allowing the pathfinding tool to run without issues.

### Running the Pathfinding Tool
Execute the pathfinding tool using the following command:

*Tabspace* python main.py --input <input_file> --search <search_algorithm>
