from Map import Map

def main():
    map = Map()
    map.get_input_from_file('input.txt')
    map.print_map()
    print(map.a_star_search())
    print(map.breadth_first_search_with_diagonal_cost())
    
if __name__ == '__main__':
    main()