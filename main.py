from Map import Map

def main():
    map = Map()
    map.get_input_from_file('input.txt')
    
    print(map.dijkstra_search_with_stops())
    map.print_map()
    
if __name__ == '__main__':
    main()