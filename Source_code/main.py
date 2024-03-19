import sys
import os
import pygame
from Map import Map
from Visualizer import Visualizer
from AStarSearch import AStarSearch
from GreedyBestFirstSearch import GreedyBestFirstSearch
from DijkstraSearch import DijkstraSearch
from AStarDynamic import AStarDynamic
import argparse  # Import the argparse module


def run_search_algorithm(algorithm_name, map_instance, visualizer):
    algorithm_name = algorithm_name.lower()  # Normalize input
    if algorithm_name == "astar":
        AStarSearch(map_instance, visualizer).a_star_search()
    elif algorithm_name == "greedy":
        GreedyBestFirstSearch(map_instance, visualizer).greedy_best_first_search()
    elif algorithm_name == "dijkstra":
        DijkstraSearch(map_instance, visualizer).dijkstra_search()
    elif algorithm_name == "dynamic":
        AStarDynamic(map_instance, visualizer, step_threshold=10).a_star_search()
    else:
        print("Unsupported search algorithm specified.")

    visualizer.run_event_loop()


def main():
    # Create the parser
    parser = argparse.ArgumentParser(description="""Run a pathfinding algorithm on a specified map.
Usage: python main.py --input <input_file> --search <search_algorithm>
Search Types:
- astar: A* Search Algorithm
- greedy: Greedy Best First Search Algorithm
- dijkstra: Dijkstra's Algorithm
- dynamic: A Dynamic version of the A* Search Algorithm""",
                                     formatter_class=argparse.RawTextHelpFormatter)  # Use RawTextHelpFormatter for a better display of the description

    # Add the arguments
    parser.add_argument('--input', '-i', type=str, required=True, help="Input file containing the map layout.")
    parser.add_argument('--search', '-s', type=str, required=True, choices=['astar', 'greedy', 'dijkstra', 'dynamic'],
                        help="Search algorithm to use. Choices: astar, greedy, dijkstra, dynamic.")

    # Execute the parse_args() method
    args = parser.parse_args()

    input_file_name = args.input
    search_algorithm = args.search

    if not os.path.exists(input_file_name):
        print(f"Error: Input file '{input_file_name}' not found.")
        sys.exit(1)

    if not pygame.init():
        print("Failed to initialize Pygame.")
        sys.exit(1)

    try:
        map_instance = Map()
        map_instance.get_input_from_file(input_file_name)
        visualizer = Visualizer(map_instance)
    except Exception as e:
        print(f"Failed to initialize map or visualizer: {e}")
        sys.exit(1)

    clock = pygame.time.Clock()
    clock.tick(60)

    run_search_algorithm(search_algorithm, map_instance, visualizer)


if __name__ == "__main__":
    main()
