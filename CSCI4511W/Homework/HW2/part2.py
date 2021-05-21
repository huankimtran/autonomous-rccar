import networkx as nx
import matplotlib.pyplot as plt
from matplotlib import lines
from ipywidgets import interact
import ipywidgets as widgets
from IPython.display import display
import time
from search import *

# Redefine the searcher function
def compare_searchers(problems, header,
                      searchers=[breadth_first_tree_search,
                                 breadth_first_graph_search,
                                 depth_first_graph_search,
                                 iterative_deepening_search,
                                 depth_limited_search,
                                 recursive_best_first_search]):
    def do(searcher, problem):
        p = InstrumentedProblem(problem)
        # Save the node to find path cost
        n = searcher(p)
        return f'Performance={p}, Cost={n.path_cost}, Path={n.solution()}'
    table = [[name(s)] + [do(s, p) for p in problems] for s in searchers]
    print_table(table, header)

# Define problems
Arad_Bucharest = GraphProblem('Arad', 'Bucharest', romania_map)
Sibiu_Bucharest = GraphProblem('Sibiu', 'Bucharest', romania_map)
Eforie_Timisoara = GraphProblem('Eforie', 'Timisoara', romania_map)
# Compare searches
compare_searchers(problems=[Arad_Bucharest, Sibiu_Bucharest, Eforie_Timisoara],
                  header=['Searcher', 'Arad to Bucharest problem', 'Sibiu to Bucharest problem', 'Eforie to Timisoara problem'],
                  searchers=[breadth_first_tree_search,
                             breadth_first_graph_search,
                             depth_first_graph_search,
                             uniform_cost_search,
                             astar_search])
