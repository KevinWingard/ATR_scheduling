from z3 import *
import networkx as nx
from funzioni_di_supporto import json_parser
from itertools import combinations

from routing_multi_depot_model import routing
from assignment_model_2 import assignment
from scheduling_model_2 import schedule
from path_changer_2 import changer
from route_checker import routes_checking

problem = 'MM_1535_0_15_5_bis'
# problem = 'MM_2547_0_20_5_bis'
# problem = 'MM_3568_0_30_5_bis'

# first of all, let's parse the json file with the plant layout and the jobs info
jobs, nodes, edges, Autonomy, ATRs, charging_coefficient = json_parser('test_cases/%s.json' % problem)

vehicles = sum(ATRs[i]['units'] for i in ATRs)

# now let's build the graph out of nodes and edges
graph = nx.DiGraph()
graph.add_nodes_from(nodes)
graph.add_weighted_edges_from([
    (i[0], i[1], edges[i][0]) for i in edges
])

# i am flattening the jobs and their task to calculate distances between any two interest point
tasks = {j + '_' + i: jobs[j]['tasks'][i]['location'] for j in jobs for i in jobs[j]['tasks']}
combination = {i: (tasks[i[0]], tasks[i[1]]) for i in combinations(tasks, 2)}


# here I compute the shortest paths between any two customers and I assign them as the current paths
current_path = {
    (i[0], i[1]): nx.shortest_path(graph, combination[i][0], combination[i][1], weight='weight')
    for i in combination
}
current_path.update({
    (i[1], i[0]): nx.shortest_path(graph, combination[i][1], combination[i][0], weight='weight')
    for i in combination
    # if k_shortest_paths(graph,combination[i][0],combination[i][1],K,weight='weight') != []
})

# for the first iteration there are no previous routes
previous_routes = []

counter = 1
RF = unknown
# while RF != unsat:
RF,RP,CS = routing(edges,jobs,tasks,Autonomy,current_path,previous_routes)
# print('solution ', counter, RF)
for i in RP:
    print(i)
previous_routes.append(CS)
counter += 1

AF,locations = assignment(ATRs,RP,charging_coefficient,jobs,current_path)
# print(AF)
# for i in locations:
#     print(i,locations[i])

# SF,nodes_schedule,edges_schedule = schedule(locations,edges)
# print(SF)

# this dictonary contains the paths used to traverse the routes
paths_combo = {
    route_index: {
        ( tasks[first], tasks[second] ):
            current_path[(first,second)] for first,second in zip(route[0][:-1],route[0][1:])
    }
        for route_index,route in enumerate(RP)
}
print('paths combo')
for i in paths_combo:
    print(i,paths_combo[i])

# convert the shortest path solution into a fromat that can be inputed into the path_changing function
shortest_paths_solution = [
            ( route, pair, (first,second))
                    for route in paths_combo
                        for pair_index,pair in enumerate(paths_combo[route])
                            for first,second in zip(paths_combo[route][pair][:-1],paths_combo[route][pair][1:])
            ]


previous_solutions = [shortest_paths_solution]
PCF = unknown
# while PCF != unsat:
PCF,path_changing_sol = changer(graph,paths_combo,previous_solutions)
previous_solutions.append(path_changing_sol)
# print(path_changing_sol)

# just an intermediate step to convert the paths_changing solution into a format readable by the route_checker
buffer = {
    route:{
        pair:[sol[2] for sol in path_changing_sol if sol[0] == route and sol[1] == pair]
        for pair in paths_combo[route]
    }
    for route in paths_combo
}
# print("buffer")
# for i in buffer:
#     print(i,buffer[i])

new_paths = {}
for route in buffer:
    new_paths.update({route:{}})
    for pair in buffer[route]:
        sequence = list(buffer[route][pair])
        path = [pair[0]]
        for _ in range(len(sequence)):
            for i in sequence:
                if i[0] == path[-1]:
                    path.append(i[1])
        # print(route,pair,path)
        new_paths[route].update({pair:path})

print('new paths')
for i in new_paths:
    print(i, new_paths[i])
# print('#####################')

CF,routes_plus = routes_checking(edges,jobs,tasks,Autonomy,new_paths,RP)
print(CF)
for i in routes_plus:
    print(i)