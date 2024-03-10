from Map import Map
from Visualizer import Visualizer
from AStarSearch import AStarSearch
from GreedyBestFirstSearch import GreedyBestFirstSearch
from DijkstraSearch import DijkstraSearch
def main():
    # Create an instance of the Map, Visualizer, and AStarSearch classes
    map_instance = Map()
    map_instance.get_input_from_file("input.txt")
    visualizer = Visualizer(map_instance)
    # a_star = AStarSearch(map_instance, visualizer).a_star_search()
    # gbfs=GreedyBestFirstSearch(map_instance,visualizer).greedy_best_first_search()
    dijkstra=DijkstraSearch(map_instance,visualizer).dijkstra_search()
    visualizer.run_event_loop()

if __name__ == "__main__":
    main()
