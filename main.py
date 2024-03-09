from Map import Map

def main():
    map = Map()
    map.get_input_from_file('input.txt')
    for i in range(5):
        map.move_polygons()
    print(map.a_star_search())
    map.draw_with_tkinter()
    
if __name__ == '__main__':
    main()