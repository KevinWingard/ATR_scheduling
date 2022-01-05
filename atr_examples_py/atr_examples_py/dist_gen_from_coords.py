# This function generates distances based on node coordinates and saves in json file for comsat
import json

def euclidian_distance(x0,y0,x1,y1):
    return ((x1-x0)**2+(y1-y0)**2)**(1/2)

def main(args=None):
    node_coord_file_path="/home/kevin/ros2/workspaces/ATR_interfaces_ws/src/atr_examples_py/config/nodes_Volvo.json"
    json_template_path="/home/kevin/ros2/workspaces/ATR_interfaces_ws/src/atr_examples_py/atr_examples_py/VRP/test_cases/Volvo_test_case_1_copy.json"

    name_new_file="Volvo_test_case_decimals_5"

    new_json_file_path=f"/home/kevin/ros2/workspaces/ATR_interfaces_ws/src/atr_examples_py/atr_examples_py/VRP/test_cases/{name_new_file}.json"

    with open(node_coord_file_path,'r') as read_file:
        coords_data = json.load(read_file)

    with open(json_template_path,'r') as read_file:
        template_data = json.load(read_file)
    
    new_data=template_data
    # print(template_data)

    # for node in coords_data["nodeTo2DCoord"]["nodes"].items():
    #     print(node[1]["x"])
    
    # print(coords_data["nodeTo2DCoord"]["nodes"][str(1)])

    for edge in template_data["edges"]:
        splitted_edge= edge.split(",") # Split string
        print(splitted_edge)
        x0=coords_data["nodeTo2DCoord"]["nodes"][splitted_edge[0]]["x"]
        y0=coords_data["nodeTo2DCoord"]["nodes"][splitted_edge[0]]["y"]
        x1=coords_data["nodeTo2DCoord"]["nodes"][splitted_edge[1]]["x"]
        y1=coords_data["nodeTo2DCoord"]["nodes"][splitted_edge[1]]["y"]
        new_data["edges"][edge][0]=round(euclidian_distance(x0,y0,x1,y1),5)

    print(new_data)

    with open(new_json_file_path, 'w') as outfile:
        json.dump(new_data, outfile, indent=4)


if __name__ == '__main__':
    main()