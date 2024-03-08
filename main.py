from Map import Map

def main():
    map = Map()
    map.get_input_from_file('input.txt')
    map.print_map()
    print(map.a_star_search())
    
if __name__ == '__main__':
    main()