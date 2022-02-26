import sys
from copy import deepcopy
from tkinter import *

#from search import *

from search import Node, deque, UndirectedGraph, Problem, PortProblem

from utils import *


from utils import PriorityQueue

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

root = None
city_coord = {}
romania_problem = None
algo = None
start = None
goal = None
counter = -1
city_map = None
frontier = None
front = None
node = None
next_button = None
explored = None

port_map = UndirectedGraph(dict(
    Lima=dict(San_Francisco=314, New_Orleans=296, Rio_de_Janeiro=409, Sydney=826, Shanghai=872, Yokohama=863, Singapore=917),
    Vancouver=dict(San_Francisco=182, Yokohama=851),
    San_Francisco=dict(New_Orleans=250, Yokohama=802, Sydney=1156),
    New_York=dict(New_Orleans=134),
    Capetown=dict(Rio_de_Janeiro=200, Dakar=350, Sydney=593),
    Dakar=dict(Rio_de_Janeiro=308, New_Orleans=248, New_York=259, Rotterdam=280, Port_Said=300),
    Port_Said=dict(Rotterdam=275),
    Rotterdam=dict(Archangelsk=300),
    Mumbai=dict(Port_Said=320, Capetown=483, Singapore=194, Sydney=549),
    Singapore=dict(Shanghai=201, Sydney=326),
    Yokohama=dict(Shanghai=237, Sydney=634),
    Shanghai=dict(Sydney=508)))
port_map.locations = dict(
    Lima=(150, 300),
    Sydney=(612, 220),
    Vancouver=(81, 571),
    New_Orleans=(157, 457),
    San_Francisco=(100, 491),
    New_York=(200, 500),
    Rio_de_Janeiro=(230, 250),
    Capetown=(380, 230),
    Dakar=(300, 400),
    Rotterdam=(345, 535),
    Archangelsk=(440, 580),
    Port_Said=(390, 490),
    Mumbai =(480, 420),
    Singapore=(530, 370),
    Shanghai=(580, 425),
    Yokohama=(620, 490))


def create_map(root):
    """Nupiešiamas žemėlapis."""
    global city_map, start, goal
    port_locations = port_map.locations
    width = 750
    height = 670
    margin = 5
    city_map = Canvas(root, width=width, height=height)
    city_map.pack()

    make_line(
        city_map,
        port_locations['Lima'][0],
        height -
        port_locations['Lima'][1],
        port_locations['Sydney'][0],
        height -
        port_locations['Sydney'][1],
        port_map.get('Lima', 'Sydney'),
        "lime")
    make_line(
        city_map,
        port_locations['San_Francisco'][0],
        height -
        port_locations['San_Francisco'][1],
        port_locations['Sydney'][0],
        height -
        port_locations['Sydney'][1],
        port_map.get('San_Francisco', 'Sydney'),
        "lime")
    make_line(
        city_map,
        port_locations['San_Francisco'][0],
        height -
        port_locations['San_Francisco'][1],
        port_locations['Yokohama'][0],
        height -
        port_locations['Yokohama'][1],
        port_map.get('San_Francisco', 'Yokohama'),
        "lime")
    make_line(
        city_map,
        port_locations['Lima'][0],
        height -
        port_locations['Lima'][1],
        port_locations['Shanghai'][0],
        height -
        port_locations['Shanghai'][1],
        port_map.get('Lima', 'Shanghai'),
        "lime")
    make_line(
        city_map,
        port_locations['Lima'][0],
        height -
        port_locations['Lima'][1],
        port_locations['Yokohama'][0],
        height -
        port_locations['Yokohama'][1],
        port_map.get('Lima', 'Yokohama'),
        "lime")
    make_line(
        city_map,
        port_locations['Lima'][0],
        height -
        port_locations['Lima'][1],
        port_locations['Singapore'][0],
        height -
        port_locations['Singapore'][1],
        port_map.get('Lima', 'Singapore'),
        "lime")
    make_line(
        city_map,
        port_locations['Vancouver'][0],
        height -
        port_locations['Vancouver'][1],
        port_locations['Yokohama'][0],
        height -
        port_locations['Yokohama'][1],
        port_map.get('Vancouver', 'Yokohama'),
        "lime")
    make_line(
        city_map,
        port_locations['Lima'][0],
        height -
        port_locations['Lima'][1],
        port_locations['Rio_de_Janeiro'][0],
        height -
        port_locations['Rio_de_Janeiro'][1],
        port_map.get('Lima', 'Rio_de_Janeiro'),
        "black")
    make_line(
        city_map,
        port_locations['Mumbai'][0],
        height -
        port_locations['Mumbai'][1],
        port_locations['Port_Said'][0],
        height -
        port_locations['Port_Said'][1],
        port_map.get('Mumbai', 'Port_Said'),
        "black")
    make_line(
        city_map,
        port_locations['Mumbai'][0],
        height -
        port_locations['Mumbai'][1],
        port_locations['Singapore'][0],
        height -
        port_locations['Singapore'][1],
        port_map.get('Mumbai', 'Singapore'),
        "black")
    make_line(
        city_map,
        port_locations['Sydney'][0],
        height -
        port_locations['Sydney'][1],
        port_locations['Singapore'][0],
        height -
        port_locations['Singapore'][1],
        port_map.get('Sydney', 'Singapore'),
        "black")
    make_line(
        city_map,
        port_locations['Sydney'][0],
        height -
        port_locations['Sydney'][1],
        port_locations['Shanghai'][0],
        height -
        port_locations['Shanghai'][1],
        port_map.get('Sydney', 'Shanghai'),
        "black")
    make_line(
        city_map,
        port_locations['Sydney'][0],
        height -
        port_locations['Sydney'][1],
        port_locations['Yokohama'][0],
        height -
        port_locations['Yokohama'][1],
        port_map.get('Sydney', 'Yokohama'),
        "black")
    make_line(
        city_map,
        port_locations['Shanghai'][0],
        height -
        port_locations['Shanghai'][1],
        port_locations['Singapore'][0],
        height -
        port_locations['Singapore'][1],
        port_map.get('Shanghai', 'Singapore'),
        "black")
    make_line(
        city_map,
        port_locations['Shanghai'][0],
        height -
        port_locations['Shanghai'][1],
        port_locations['Yokohama'][0],
        height -
        port_locations['Yokohama'][1],
        port_map.get('Shanghai', 'Yokohama'),
        "black")
    make_line(
        city_map,
        port_locations['Mumbai'][0],
        height -
        port_locations['Mumbai'][1],
        port_locations['Capetown'][0],
        height -
        port_locations['Capetown'][1],
        port_map.get('Mumbai', 'Capetown'),
        "black")
    make_line(
        city_map,
        port_locations['Mumbai'][0],
        height -
        port_locations['Mumbai'][1],
        port_locations['Sydney'][0],
        height -
        port_locations['Sydney'][1],
        port_map.get('Mumbai', 'Sydney'),
        "black")
    make_line(
        city_map,
        port_locations['Capetown'][0],
        height -
        port_locations['Capetown'][1],
        port_locations['Sydney'][0],
        height -
        port_locations['Sydney'][1],
        port_map.get('Capetown', 'Sydney'),
        "black")
    make_line(
        city_map,
        port_locations['Capetown'][0],
        height -
        port_locations['Capetown'][1],
        port_locations['Dakar'][0],
        height -
        port_locations['Dakar'][1],
        port_map.get('Capetown', 'Dakar'),
        "black")
    make_line(
        city_map,
        port_locations['Rotterdam'][0],
        height -
        port_locations['Rotterdam'][1],
        port_locations['Port_Said'][0],
        height -
        port_locations['Port_Said'][1],
        port_map.get('Rotterdam', 'Port_Said'),
        "black")
    make_line(
        city_map,
        port_locations['Dakar'][0],
        height -
        port_locations['Dakar'][1],
        port_locations['Port_Said'][0],
        height -
        port_locations['Port_Said'][1],
        port_map.get('Dakar', 'Port_Said'),
        "black")
    make_line(
        city_map,
        port_locations['Archangelsk'][0],
        height -
        port_locations['Archangelsk'][1],
        port_locations['Rotterdam'][0],
        height -
        port_locations['Rotterdam'][1],
        port_map.get('Archangelsk', 'Rotterdam'),
        "black")
    make_line(
        city_map,
        port_locations['New_York'][0],
        height -
        port_locations['New_York'][1],
        port_locations['Rotterdam'][0],
        height -
        port_locations['Rotterdam'][1],
        port_map.get('New_York', 'Rotterdam'),
        "black")
    make_line(
        city_map,
        port_locations['Dakar'][0],
        height -
        port_locations['Dakar'][1],
        port_locations['Rotterdam'][0],
        height -
        port_locations['Rotterdam'][1],
        port_map.get('Dakar', 'Rotterdam'),
        "black")
    make_line(
        city_map,
        port_locations['Dakar'][0],
        height -
        port_locations['Dakar'][1],
        port_locations['Rio_de_Janeiro'][0],
        height -
        port_locations['Rio_de_Janeiro'][1],
        port_map.get('Rio_de_Janeiro', 'Dakar'),
        "black")
    make_line(
        city_map,
        port_locations['Dakar'][0],
        height -
        port_locations['Dakar'][1],
        port_locations['New_Orleans'][0],
        height -
        port_locations['New_Orleans'][1],
        port_map.get('New_Orleans', 'Dakar'),
        "black")
    make_line(
        city_map,
        port_locations['Dakar'][0],
        height -
        port_locations['Dakar'][1],
        port_locations['New_York'][0],
        height -
        port_locations['New_York'][1],
        port_map.get('New_York', 'Dakar'),
        "black")
    make_line(
        city_map,
        port_locations['Capetown'][0],
        height -
        port_locations['Capetown'][1],
        port_locations['Rio_de_Janeiro'][0],
        height -
        port_locations['Rio_de_Janeiro'][1],
        port_map.get('Capetown', 'Rio_de_Janeiro'),
        "black")
    make_line(
        city_map,
        port_locations['Lima'][0],
        height -
        port_locations['Lima'][1],
        port_locations['San_Francisco'][0],
        height -
        port_locations['San_Francisco'][1],
        port_map.get('Lima', 'San_Francisco'),
        "black")
    make_line(
        city_map,
        port_locations['New_Orleans'][0],
        height -
        port_locations['New_Orleans'][1],
        port_locations['New_York'][0],
        height -
        port_locations['New_York'][1],
        port_map.get('New_Orleans', 'New_York'),
        "black")
    make_line(
        city_map,
        port_locations['Lima'][0],
        height -
        port_locations['Lima'][1],
        port_locations['New_Orleans'][0],
        height -
        port_locations['New_Orleans'][1],
        port_map.get('Lima', 'New_Orleans'),
        "black")
    make_line(
        city_map,
        port_locations['Vancouver'][0],
        height -
        port_locations['Vancouver'][1],
        port_locations['San_Francisco'][0],
        height -
        port_locations['San_Francisco'][1],
        port_map.get('Vancouver', 'San_Francisco'),
        "black")
    make_line(
        city_map,
        port_locations['San_Francisco'][0],
        height -
        port_locations['San_Francisco'][1],
        port_locations['New_Orleans'][0],
        height -
        port_locations['New_Orleans'][1],
        port_map.get('San_Francisco', 'New_Orleans'),
        "black")


    for city in port_locations.keys():
        make_rectangle(
            city_map,
            port_locations[city][0],
            height -
            port_locations[city][1],
            margin,
            city)

    make_legend(city_map)


def make_line(map, x0, y0, x1, y1, distance, color):
    """Nupiešiamos linijos."""
    map.create_line(x0, y0, x1, y1, fill=color)
    map.create_text((x0 + x1) / 2, (y0 + y1) / 2, text=distance, fill=color)


def make_rectangle(map, x0, y0, margin, city_name):
    """Nupiešiami uostų kvadratai."""
    global city_coord
    rect = map.create_rectangle(
        x0 - margin,
        y0 - margin,
        x0 + margin,
        y0 + margin,
        fill="white")
    if "Bucharest" in city_name or "Pitesti" in city_name or "Lugoj" in city_name \
            or "Mehadia" in city_name or "Drobeta" in city_name:
        map.create_text(
            x0 - 2 * margin,
            y0 - 2 * margin,
            text=city_name,
            anchor=E)
    else:
        map.create_text(
            x0 - 1 * margin,
            y0 - 1 * margin + 15,
            text=city_name,
            anchor=SE)
    city_coord.update({city_name: rect})


def make_legend(map):
    """Nupiešiami sutartiniai ženklai"""
    rect1 = map.create_rectangle(600, 80, 610, 90, fill="white")
    text1 = map.create_text(615, 85, anchor=W, text="Un-explored")

    rect2 = map.create_rectangle(600, 95, 610, 105, fill="orange")
    text2 = map.create_text(615, 100, anchor=W, text="Frontier")

    rect3 = map.create_rectangle(600, 110, 610, 120, fill="red")
    text3 = map.create_text(615, 115, anchor=W, text="Currently Exploring")

    rect4 = map.create_rectangle(600, 125, 610, 135, fill="grey")
    text4 = map.create_text(615, 130, anchor=W, text="Explored")

    rect5 = map.create_rectangle(600, 140, 610, 150, fill="dark green")
    text5 = map.create_text(615, 145, anchor=W, text="Final Solution")


def display_frontier(queue):
    """Nuspalvinami visi eilėje esantys uostai."""
    global city_map, city_coord
    qu = deepcopy(queue)
    while qu:
        node = qu.pop()
        for city in city_coord.keys():
            if node.state == city:
                city_map.itemconfig(city_coord[city], fill="orange")


def display_current(node):
    """Nuspalvinamas nagrinėjamas uostas."""
    global city_map, city_coord
    city = node.state
    city_map.itemconfig(city_coord[city], fill="red")


def display_explored(node):
    """Nuspalvinami jau išnagrinėti uostai."""
    global city_map, city_coord
    city = node.state
    city_map.itemconfig(city_coord[city], fill="gray")


def display_final(cities):
    """Nuspalvinami rasto kelio uostai."""
    global city_map, city_coord
    for city in cities:
        city_map.itemconfig(city_coord[city], fill="green")


def breadth_first_tree_search(problem):
    """Medžio paieška į plotį."""
    global frontier, counter, node
    if counter == -1:
        frontier = deque()

    if counter == -1:
        frontier.append(Node(problem.initial))

        display_frontier(frontier)
    if counter % 3 == 0 and counter >= 0:
        node = frontier.popleft()

        display_current(node)
    if counter % 3 == 1 and counter >= 0:
        if problem.goal_test(node.state):
            return node
        frontier.extend(node.expand(problem))

        display_frontier(frontier)
    if counter % 3 == 2 and counter >= 0:
        display_explored(node)
    return None


def depth_first_tree_search(problem):
    """Medžio paieška į gylį."""
    # This search algorithm might not work in case of repeated paths.
    global frontier, counter, node
    if counter == -1:
        frontier = []  # stack

    if counter == -1:
        frontier.append(Node(problem.initial))

        display_frontier(frontier)
    if counter % 3 == 0 and counter >= 0:
        node = frontier.pop()

        display_current(node)
    if counter % 3 == 1 and counter >= 0:
        if problem.goal_test(node.state):
            return node
        frontier.extend(node.expand(problem))

        display_frontier(frontier)
    if counter % 3 == 2 and counter >= 0:
        display_explored(node)
    return None


def breadth_first_graph_search(problem):
    """Paieška į plotį su jau aplankytų uostų eile."""
    global frontier, node, explored, counter
    if counter == -1:
        node = Node(problem.initial)
        display_current(node)
        if problem.goal_test(node.state):
            return node

        frontier = deque([node])  # FIFO queue

        display_frontier(frontier)
        explored = set()
    if counter % 3 == 0 and counter >= 0:
        node = frontier.popleft()
        display_current(node)
        explored.add(node.state)
    if counter % 3 == 1 and counter >= 0:
        for child in node.expand(problem):
            if child.state not in explored and child not in frontier:
                if problem.goal_test(child.state):
                    return child
                frontier.append(child)
        display_frontier(frontier)
    if counter % 3 == 2 and counter >= 0:
        display_explored(node)
    return None


def depth_first_graph_search(problem):
    """Paieška į gylį su jau aplankytų uostų eile."""
    global counter, frontier, node, explored
    if counter == -1:
        frontier = [Node(problem.initial)]
        explored = set()

        display_frontier(frontier)
    if counter % 3 == 0 and counter >= 0:
        node = frontier.pop()

        display_current(node)
    if counter % 3 == 1 and counter >= 0:
        if problem.goal_test(node.state):
            return node
        explored.add(node.state)
        frontier.extend(child for child in node.expand(problem)
                        if child.state not in explored and
                        child not in frontier)

        display_frontier(frontier)
    if counter % 3 == 2 and counter >= 0:
        display_explored(node)
    return None


def best_first_graph_search(problem, f):
    """Nagrinėjami mazgai, kurie pasiekti per trumpiausią kelią."""
    global frontier, node, explored, counter

    if counter == -1:
        f = memoize(f, 'f')
        node = Node(problem.initial)
        display_current(node)
        if problem.goal_test(node.state):
            return node
        frontier = PriorityQueue('min', f)
        frontier.append(node)
        display_frontier(frontier)
        explored = set()
    if counter % 3 == 0 and counter >= 0:
        node = frontier.pop()
        display_current(node)
        if problem.goal_test(node.state):
            return node
        explored.add(node.state)
    if counter % 3 == 1 and counter >= 0:
        for child in node.expand(problem):
            if child.state not in explored and child not in frontier:
                frontier.append(child)
            elif child in frontier:
                if f(child) < frontier[child]:
                    del frontier[child]
                    frontier.append(child)
        display_frontier(frontier)
    if counter % 3 == 2 and counter >= 0:
        display_explored(node)
    return None


def uniform_cost_search(problem):
    return best_first_graph_search(problem, lambda node: node.path_cost)


def astar_search(problem, h=None):
    """Nagrinėjamas mazgas pasirenkamas pagal mažiausią nueito ir dar likusio kelio sumą."""
    h = memoize(h or problem.h, 'h')
    return best_first_graph_search(problem, lambda n: n.path_cost + h(n))


def on_click():
    """
    Funkcija, nurodanti, ką turi daryti programa, kai paspaudžiamas mygtukas "Next".
    """
    global algo, counter, next_button, romania_problem, start, goal
    romania_problem = PortProblem(start.get(), goal.get(), port_map)
    if "Breadth-First Tree Search" == algo.get():
        node = breadth_first_tree_search(romania_problem)
        if node is not None:
            final_path = breadth_first_tree_search(romania_problem).solution()
            final_path.append(start.get())
            display_final(final_path)
            next_button.config(state="disabled")
        counter += 1
    elif "Depth-First Tree Search" == algo.get():
        node = depth_first_tree_search(romania_problem)
        if node is not None:
            final_path = depth_first_tree_search(romania_problem).solution()
            final_path.append(start.get())
            display_final(final_path)
            next_button.config(state="disabled")
        counter += 1
    elif "Breadth-First Graph Search" == algo.get():
        node = breadth_first_graph_search(romania_problem)
        if node is not None:
            final_path = breadth_first_graph_search(romania_problem).solution()
            final_path.append(start.get())
            display_final(final_path)
            next_button.config(state="disabled")
        counter += 1
    elif "Depth-First Graph Search" == algo.get():
        node = depth_first_graph_search(romania_problem)
        if node is not None:
            final_path = depth_first_graph_search(romania_problem).solution()
            final_path.append(start.get())
            display_final(final_path)
            next_button.config(state="disabled")
        counter += 1
    elif "Uniform Cost Search" == algo.get():
        node = uniform_cost_search(romania_problem)
        if node is not None:
            final_path = uniform_cost_search(romania_problem).solution()
            final_path.append(start.get())
            display_final(final_path)
            next_button.config(state="disabled")
        counter += 1
    elif "A* - Search" == algo.get():
        node = astar_search(romania_problem)
        if node is not None:
            final_path = astar_search(romania_problem).solution()
            final_path.append(start.get())
            display_final(final_path)
            next_button.config(state="disabled")
        counter += 1


def reset_map():
    """Išvalomas sprendimas"""
    global counter, city_coord, city_map, next_button
    counter = -1
    for city in city_coord.keys():
        city_map.itemconfig(city_coord[city], fill="white")
    next_button.config(state="normal")


if __name__ == "__main__":
    # global algo, start, goal, next_button
    root = Tk()
    root.title("Road Map of Romania")
    root.geometry("950x1050")
    algo = StringVar(root)
    start = StringVar(root)
    goal = StringVar(root)
    algo.set("Breadth-First Tree Search")
    start.set('Lima')
    goal.set('Archangelsk')
    cities = sorted(port_map.locations.keys())
    algorithm_menu = OptionMenu(
        root,
        algo, "Breadth-First Tree Search", "Depth-First Tree Search",
        "Breadth-First Graph Search", "Depth-First Graph Search",
        "Uniform Cost Search", "A* - Search")
    Label(root, text="\n Search Algorithm").pack()
    algorithm_menu.pack()
    Label(root, text="\n Start City").pack()
    start_menu = OptionMenu(root, start, *cities)
    start_menu.pack()
    Label(root, text="\n Goal City").pack()
    goal_menu = OptionMenu(root, goal, *cities)
    goal_menu.pack()
    frame1 = Frame(root)
    next_button = Button(
        frame1,
        width=6,
        height=2,
        text="Next",
        command=on_click,
        padx=2,
        pady=2,
        relief=GROOVE)
    # next_button.pack(side=RIGHT)
    next_button.pack(side="right")
    reset_button = Button(
        frame1,
        width=6,
        height=2,
        text="Reset",
        command=reset_map,
        padx=2,
        pady=2,
        relief=GROOVE)
    #reset_button.pack(side=RIGHT)
    reset_button.pack(side="right")
    frame1.pack(side=BOTTOM)
    create_map(root)
    root.mainloop()
    
    
# class PortProblem(Problem):
#     """The problem of searching a graph from one node to another."""
#
#     def __init__(self, initial, goal, graph):
#         super().__init__(initial, goal)
#         self.graph = graph
#
#     def actions(self, A):
#         """The actions at a graph node are just its neighbors."""
#         return list(self.graph.get(A).keys())
#
#     def result(self, state, action):
#         """The result of going to a neighbor is just that neighbor."""
#         return action
#
#     def path_cost(self, cost_so_far, A, action, B):
#         return cost_so_far + (self.graph.get(A, B) or np.inf)
#
#     def find_min_edge(self):
#         """Find minimum value of edges."""
#         m = np.inf
#         for d in self.graph.graph_dict.values():
#             local_min = min(d.values())
#             m = min(m, local_min)
#
#         return m
#
#     def h(self, node):
#         """h function is straight-line distance from a node's state to goal."""
#         locs = getattr(self.graph, 'locations', None)
#         if locs:
#             if type(node) is str:
#                 return int(distance(locs[node], locs[self.goal]))
#
#             return int(distance(locs[node.state], locs[self.goal]))
#         else:
#             return np.inf