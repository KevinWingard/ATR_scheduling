from support_functions import json_parser
import networkx as nx
import matplotlib.pyplot as plt

def draw(problem,ws_path):
    print('Draw function')
    print('instance', problem)

    # parse instance
    jobs, nodes, edges, Autonomy, ATRs, charging_coefficient, Big_number, charging_stations, hub_nodes, start_list = json_parser('%s/test_cases/%s.json' % (ws_path,problem), monolithic=True)

    # now let's build the graph out of nodes and edges
    graph = nx.Graph()
    pos_x = 0
    pos_y = 0
    dist_scale = 1
    for node in nodes:
        graph.add_node(node, pos=(pos_x, pos_y))
        pos_x += dist_scale
        if pos_x == 5 * dist_scale:
            pos_x = 0
            pos_y += dist_scale
    pos = nx.get_node_attributes(graph, 'pos')

    # graph.add_nodes_from(nodes)

    graph.add_weighted_edges_from([
        (i[0], i[1], edges[i][0]) for i in edges
    ])

    col_dict = {}
    colors = ['red', 'orange', 'green', 'gray']

    labels = ["Hub node and charging station",
              "Hub node",
              "Charging station",
              "Other node"]

    # specify color
    for node in graph.nodes:
        if node in hub_nodes and node in charging_stations:
            col_dict[node] = {'col': colors[0]}
        elif node in hub_nodes:
            col_dict[node] = {'col': colors[1]}
        elif node in charging_stations:
            col_dict[node] = {'col': colors[2]}
        else:
            col_dict[node] = {'col': colors[3]}

    # draw network
    plt.figure(figsize=(12, 8))
    ax = plt.subplot(121)
    nx.draw(graph, pos, node_size=500, ax=ax, with_labels=True, node_color=[val['col'] for _, val in col_dict.items()])

    # print legends in second subplot
    plt.subplot(122)
    plt.axis('off')
    for idx, elem in enumerate(colors):
        plt.scatter([], [], c=elem, label=labels[idx])
    plt.legend(loc=10)
    #info_str=f"Instance: {problem}\nJobs: {len(jobs)}\nATRs: {len(ATRs)}"
    #plt.text(x=1.7, y=0.65, s=info_str, horizontalalignment='center', verticalalignment = 'center',
    #         transform = ax.transAxes,bbox=dict(facecolor='gray', alpha=0.5))

    plt.show()
