from Map import Map

def main():
    map = Map()
    if(map.get_input_from_file('input.txt')):
        print(map.greedy_best_first_search())
        map.print_map()
    
if __name__ == '__main__':
    main()