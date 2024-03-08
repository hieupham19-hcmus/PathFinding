from Map import Map

def main():
    map = Map()
    map.get_input_from_file('input.txt')
    map.move_polygons()
    map.move_polygons()
    map.move_polygons() 
    map.move_polygons()
    map.move_polygons()
    map.print_map()
    
if __name__ == '__main__':
    main()