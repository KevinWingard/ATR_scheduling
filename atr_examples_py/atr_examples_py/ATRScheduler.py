from atr_interfaces.srv import ComSat

from atr_interfaces.msg import ATRStateList
from atr_interfaces.msg import ATRStateMission

from atr_interfaces.msg import ATRStateListStamped
from atr_interfaces.msg import ATRTimeNodeListArray
from atr_interfaces.msg import ATRTimeNodeList
from atr_interfaces.msg import ATRTimeNode

from atr_interfaces.msg import ATRPath
from atr_interfaces.msg import ATRPathList
from atr_interfaces.msg import ATRPathStamped
from atr_interfaces.msg import ATRPath
from atr_interfaces.msg import PathWithDTime
from atr_interfaces.msg import PoseWithDTime

from atr_interfaces.srv import UpdateATRPathList

from geometry_msgs.msg import Pose

import json

import numpy as np

from std_msgs.msg import String

import sys, os
sys.path.append(os.getcwd()+'/atr_examples_py/atr_examples_py/VRP') # To access VRP packages, UGLY SOLUTION, FIX SOMETIME
from Compo_algo import Compositional

import rclpy
from rclpy.node import Node

import re # regular expressions

import time


# Disable print (used when running comsat to compress prints)
def blockPrint():
    sys.stdout = open(os.devnull, 'w')

# Enable print
def enablePrint():
    sys.stdout = sys.__stdout__


class SchedulerClass(Node):

    def __init__(self):
        super().__init__('schedule_node')
        self.local_state_list = ATRStateList()
        self.state_subscriber = super().create_subscription(ATRStateListStamped,'atr_state_list',self.state_subscriber_callback,100)
        self.schedule_srv = super().create_service(
            ComSat, 'get_job_schedule', self.schedule_callback)

        self.path_publisher=super().create_publisher(ATRPathList,"atr_path_list",10)

        self.path_client=super().create_client(UpdateATRPathList,"update_path_from_scheduler")

        self.path_to_json=os.getcwd()+"/atr_examples_py/config/nodes_Volvo.json"
        with open(self.path_to_json,'r') as read_file:
            self.node_data = json.load(read_file)
         
    def run_comsat(self,problem,path):
        self.get_logger().info("Running ComSat on problem: %s" % problem)
        
        start_time = time.time()

        blockPrint() # comsat has internal prints which we can suppress while running
        node_sequence, edge_sequence,feasible,_,_,_ =Compositional(problem,path)
        enablePrint() # enable all prints again

        print("--- %s seconds ---" % (time.time() - start_time))

        self.get_logger().info("ComSat done.")

        if (str(feasible)!='sat'):
            self.get_logger().info("Problem not feasible!!")
        else:
            self.get_logger().info("Problem feasible")

        return node_sequence, edge_sequence, feasible

    def schedule_callback(self, request, response):
        problem=String()
        problem.data=request.problem
        self.get_logger().info(
        'Incoming request, problem: %s' % problem.data)
        ws_path = f"{os.getcwd()}/atr_examples_py/atr_examples_py/VRP" # ugly solution

        # Check for available ATRs
        availableATRs=self.checkForAvailableATRs() # later include argument for specifying ATR type (perhaps others parameters as well)
        print('available ATRs:',*availableATRs, sep = ", ")

        node_sequence, edge_sequence,feasible = self.run_comsat(problem.data,ws_path) # Run comsat

        if (str(feasible)!='sat'):
            response.status="Error, scheduling not feasible!"
        else:
            node_time_list,edge_time_list=self.parse_comsat_output(node_sequence, edge_sequence)
            response.status="Node list generated!"
            response.atr_node_list=node_time_list

        path_list=self.discrete_path_generator(node_time_list,edge_time_list) # generate discrete paths between nodes

        self.client_send_path_update(path_list)
        self.get_logger().info(response.status)
        return response
        
    def state_subscriber_callback(self,msg):
        self.local_state_list=msg.list
        # self.get_logger().info(__file__ +'Got state list')
        return

    def checkForAvailableATRs(self):
        # In this function, the ATR state list should be checked to see whether ATRs are available and where they are located etc.
        # Currently not used! Right now, all ATRs in the simulation are assumed to be stationary and available when publishing jobs
        availableATRs=[]
        for state in self.local_state_list.atr_states:
            print(f"Status for ATR {state.atr_id}: {state.mission.status}")
            if state.mission.status==ATRStateMission.UNDEFINED:
                availableATRs.append(state.atr_id)

        return availableATRs

    def parse_comsat_output(self,node_sequence, edge_sequence):
        # ------------ nodes
        node_time_list = ATRTimeNodeListArray()
        prev_ATR=None

        for node_tuple in node_sequence:
            splitted_tuple= node_tuple[0].split("_") # Split string
            atr_idx=splitted_tuple[1] 
            node=re.findall(r'\d+', splitted_tuple[5])[0] # Finds digit
            time=node_tuple[1]
            if atr_idx!=prev_ATR:
                if prev_ATR!=None:
                    node_time_list.atr_node_list.append(intermediate_ATR)
                prev_ATR=atr_idx
                intermediate_ATR =ATRTimeNodeList()
                intermediate_ATR.atr_id=int(atr_idx)
    
            intermediate_node=ATRTimeNode()
            intermediate_node.node=int(node)
            intermediate_node.time_step=round(float(time.as_fraction().numerator)/float(time.as_fraction().denominator),1)
            intermediate_ATR.nodes.append(intermediate_node)
        node_time_list.atr_node_list.append(intermediate_ATR) # add last one

        # ------------ edges
        edge_time_list = ATRTimeNodeListArray()
        prev_ATR=None

        for edge_tuple in edge_sequence:
            splitted_tuple= edge_tuple[0].split("_") # Split string
            atr_idx=splitted_tuple[1] 
            node=re.findall(r'\d+', splitted_tuple[5])[0] # Finds digit (not used for edges)
            time=edge_tuple[1]
            if atr_idx!=prev_ATR:
                if prev_ATR!=None:
                    edge_time_list.atr_node_list.append(intermediate_ATR)
                prev_ATR=atr_idx
                intermediate_ATR =ATRTimeNodeList()
                intermediate_ATR.atr_id=int(atr_idx)
    
            intermediate_node=ATRTimeNode()
            intermediate_node.node=int(node)
            intermediate_node.time_step=round(float(time.as_fraction().numerator)/float(time.as_fraction().denominator),1)
            intermediate_ATR.nodes.append(intermediate_node)
        edge_time_list.atr_node_list.append(intermediate_ATR) # add last one

        print("\nParsed edge and node sequences:")
        for atr_idx,atr in enumerate(edge_time_list.atr_node_list):
            print(f"\nATR id: {atr.atr_id}")
            # for node_idx,node in enumerate(atr.nodes):
            for node_idx in range(len(atr.nodes)):
                print(f"visits node: ({node_time_list.atr_node_list[atr_idx].nodes[node_idx].node}) at time: {node_time_list.atr_node_list[atr_idx].nodes[node_idx].time_step} and entering edge: ({node_time_list.atr_node_list[atr_idx].nodes[node_idx].node},{node_time_list.atr_node_list[atr_idx].nodes[node_idx+1].node}) at time: {edge_time_list.atr_node_list[atr_idx].nodes[node_idx].time_step}")
            print(f"finished at node ({node_time_list.atr_node_list[atr_idx].nodes[-1].node}) at time {node_time_list.atr_node_list[atr_idx].nodes[-1].time_step}")
        return node_time_list,edge_time_list

    def discrete_path_generator(self,node_time_list,edge_time_list):
        sample_t=0.1

        path_list=ATRPathList() # To be returned
        for atr_idx,atr in enumerate(node_time_list.atr_node_list):
            print("\n###########################################")
            print("working with atr" ,atr.atr_id)
            time_count=0
            
            atr_path_list_stamped=ATRPathStamped() # header + ATRPath

            intermediate_atr_path=ATRPath() # to be added in atr_path_list_stamped (contains id, priority and pathwithDtime) 
            intermediate_atr_path.atr_id=atr.atr_id
            intermediate_atr_path.priority=1 # or what?

            pose_counter=0 # just to see how many poses are generated for each atr

            # Generate path
            pathwithDtime=PathWithDTime()
            
            for index,_ in enumerate(atr.nodes):
                print('time count', time_count)
                if (index==0):
                    
                    # Add first pose/position (no trajectory for first node)
                    # Force ATRs to initial position
                    tmp_pose=Pose()
                    tmp_pose._position.x=float(self.node_data["nodeTo2DCoord"]["nodes"][str(atr.nodes[0].node)]["x"])
                    tmp_pose._position.y=float(self.node_data["nodeTo2DCoord"]["nodes"][str(atr.nodes[0].node)]["y"])

                    posewithDtime=PoseWithDTime()
                    
                    posewithDtime.pose=tmp_pose
                    posewithDtime.delta_time=0.0

                    pathwithDtime.poses.append(posewithDtime)
                    pose_counter+=1

                else:
                    # add trajectories between the rest of the nodes
                    x_start=float(self.node_data["nodeTo2DCoord"]["nodes"][str(atr.nodes[index-1].node)]["x"])
                    y_start=float(self.node_data["nodeTo2DCoord"]["nodes"][str(atr.nodes[index-1].node)]["y"])
                    x_end=float(self.node_data["nodeTo2DCoord"]["nodes"][str(atr.nodes[index].node)]["x"])
                    y_end=float(self.node_data["nodeTo2DCoord"]["nodes"][str(atr.nodes[index].node)]["y"])

                    # print(x_start,y_start,x_end,y_end)
                    xtraj,ytraj = self.generateTrajBetw2Nodes(x_start,y_start,x_end,y_end,sample_t)
                    print("traj length ",len(xtraj))
                    print(xtraj[0],xtraj[-1])
                    print(ytraj[0],ytraj[-1])
                    print(f"init traj from {str(atr.nodes[index-1].node)} to {str(atr.nodes[index].node)} at time {round(time_count,2)}")
                    for idx,_ in enumerate(xtraj):
                        # add points from generated trajectory
                        tmp_pose=Pose()
                        tmp_pose._position.x=float(xtraj[idx])
                        tmp_pose._position.y=float(ytraj[idx])

                        if idx==len(xtraj)-1: # check whether ATR should slow down for service stop or continue
                            if index==len(node_time_list.atr_node_list[atr_idx].nodes)-1: # check if next node is last node
                                diff_t=0.1
                            else:
                                diff_t=round(edge_time_list.atr_node_list[atr_idx].nodes[index].time_step-time_count,1)
                                print('diff ',diff_t,' edgetime ',round(edge_time_list.atr_node_list[atr_idx].nodes[index].time_step,1),' time_count ',round(time_count,1))
                            time_count+=diff_t
                            posewithDtime.delta_time=float(diff_t)
                        else:
                            posewithDtime.delta_time=float(sample_t)
                            posewithDtime=PoseWithDTime()
                            posewithDtime.pose=tmp_pose
                            time_count+=sample_t
                            pathwithDtime.poses.append(posewithDtime)
                            pose_counter+=1
                    time_count=round(time_count,1) # make sure it only has one decimal
                    
                    pathwithDtime.poses.append(posewithDtime)
                    pose_counter+=1
                
                intermediate_atr_path._path_w_time=pathwithDtime

            atr_path_list_stamped.atr_path=intermediate_atr_path
            print(f"number of poses added for atr {atr.atr_id}: {pose_counter}")
            pose_counter=0
            path_list.paths.append(atr_path_list_stamped)

        return path_list

    def generateTrajBetw2Nodes(self,x_start, y_start, x_end, y_end, sample_t):
        # Speed of ATR
        vel = 1

        # distance d between start and end node. (Pythagoras)
        distX = x_end - x_start                 # Distance x-dimension
        distY = y_end - y_start                 # Distance y-dimension
        d = (distX**2 + distY**2)**.5

        # t, required time to complete travel
        t = d*vel

        # length of the lists requierd to fit data (round up)
        # length = int(t/sample_t) + (t % sample_t > 0)
        length=int(np.floor(t/sample_t))

        # delta distances (x and y), required step length per sample
        deltaX = distX/length
        deltaY = distY/length

        # create empty lists with correct lengths
        xtraj = [None] * length
        ytraj = [None] * length

        for i in range(length):
            if i != (length-1):                                         # For all element except the last
                xtraj[i] = float(round((i+1)*deltaX + x_start,4))
                ytraj[i] = float(round((i+1)*deltaY + y_start,4))
            else:
                xtraj[i] = x_end                      # Setting the last element in lists to
                ytraj[i] = y_end                      # to the coordinate of the end node
        return xtraj,ytraj

    def client_send_path_update(self,path_list):
        node = rclpy.create_node('minimal_client_async')
        cli = node.create_client(UpdateATRPathList, 'update_path_from_scheduler')
        req = UpdateATRPathList.Request()
        req.list=path_list
        while not cli.wait_for_service(timeout_sec=1.0):
            node.get_logger().info('service not available, waiting again...')

        future = cli.call_async(req)
        while rclpy.ok():
            rclpy.spin_once(node)
            if future.done():
                result = future.result()
                node.get_logger().info(
                    'Success response of path updater: %s' %
                    (result.success))
                break

        node.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    scheduler=SchedulerClass()
    rclpy.spin(scheduler)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
