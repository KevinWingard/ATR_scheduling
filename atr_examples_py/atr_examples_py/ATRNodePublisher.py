from atr_interfaces import msg
import rclpy
from rclpy.node import Node

from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker

import json
import os


class NodePublisher(Node):

    def __init__(self):

        self.path_to_json=os.getcwd()+"/atr_examples_py/config/nodes_Volvo.json"
        super().__init__('node_publisher')
        self.publisher_ = self.create_publisher(
            MarkerArray, 'visualization_marker_array', 10)
        timer_period = 2
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.markers=MarkerArray()
        self.init_array()


    def timer_callback(self):      
        self.publisher_.publish(self.markers)

    def init_array(self):
   
        with open(self.path_to_json,'r') as read_file:
            data = json.load(read_file)
        for _,(node,coords) in enumerate(data['nodeTo2DCoord']['nodes'].items()):
            marker_base=Marker()
            marker_base.header.frame_id="map"
            marker_base.type=Marker.CYLINDER
            marker_base.action=Marker.ADD
            marker_base.pose.orientation.x = 0.0
            marker_base.pose.orientation.y = 0.0
            marker_base.pose.orientation.z = 0.0
            marker_base.pose.orientation.w = 1.0
            marker_base.scale.x = 0.5
            marker_base.scale.y = 0.5
            marker_base.scale.z = 0.1
            if (int(node) in data["info"]["charging_nodes"]):
                marker_base.color.r=0.0
                marker_base.color.g=1.0
                marker_base.color.b=0.0
            elif (int(node) in data["info"]["hub_nodes"]):
                print("true")
                marker_base.color.r=1.0
                marker_base.color.g=0.5
                marker_base.color.b=0.0
            else:
                marker_base.color.r = 0.0
                marker_base.color.g = 0.0
                marker_base.color.b = 1.0

            marker_base.color.a = 1.0

            marker_base.pose.position.z = 0.0
            marker_base.id=int(node)+1000 # different from ids for number shapes below
            marker_base.ns="nodes_circles"
            marker_base.pose.position.x=float(coords["x"])
            marker_base.pose.position.y=float(coords["y"])
            self.markers.markers.append(marker_base)
            marker_base=Marker()
            marker_base.header.frame_id="map"
            marker_base.type=Marker.TEXT_VIEW_FACING
            marker_base.action=Marker.ADD
            marker_base.pose.orientation.x = 0.0
            marker_base.pose.orientation.y = 0.0
            marker_base.pose.orientation.z = 0.0
            marker_base.pose.orientation.w = 1.0
            marker_base.scale.x = 0.4
            marker_base.scale.y = 0.4
            marker_base.scale.z = 0.4
            marker_base.color.a = 1.0
            marker_base.color.r = 1.0
            marker_base.color.g = 1.0
            marker_base.color.b = 1.0
            marker_base.pose.position.z = 0.05
            marker_base.id=int(node)
            marker_base.ns="nodes"
            marker_base.text=node
            marker_base.pose.position.x=float(coords["x"])
            marker_base.pose.position.y=float(coords["y"]-0.05)
            self.markers.markers.append(marker_base)
        return


def main(args=None):
    rclpy.init(args=args)

    node_publisher = NodePublisher()

    rclpy.spin(node_publisher)

    node_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
