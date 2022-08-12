import random
import time
import numpy as np
from functools import partial
import os
from multiprocessing import Lock

import rclpy
from rclpy.node import Node

from gazebo_msgs.msg import ModelStates

from geometry_msgs.msg import PoseStamped

class RateLimiter():
    def __init__(self,min_period,clock):
        self.min_period = rclpy.duration.Duration(seconds=min_period)
        self.clock = clock
        self.time_of_last = None

    def call(self,func):
        if self.time_of_last is None:
            self.time_of_last = self.clock.now()
            time_since_last = self.min_period
        else:
            time_since_last = self.clock.now() - self.time_of_last

        if time_since_last > self.min_period:
            func()

class Monitor(Node):

    def __init__(self):
        super().__init__('motion_tracker_sim')

        self.declare_parameter('model_prefixes', os.getenv("MOTION_TRACKER_ENTITY_PREFIX_LIST", "iris gimbal rover"))
        self.declare_parameter('rate_limit', 30)
        self.declare_parameter('pose_topic', "mavros/vision_pose/pose")
        self.declare_parameter('world_frame_id', 'map')

        self.pose_topic = self.get_parameter('pose_topic').value
        self.frame_id = self.get_parameter('world_frame_id').value

        self.pose_rate_limiter = RateLimiter(self.get_parameter('rate_limit').value, self.get_clock())
        self.gazebo_model_state_sub = self.create_subscription(ModelStates, '/model_states',
                lambda msg: self.pose_rate_limiter.call(lambda: self.model_states_cb(msg)), 10)

        self.current_states_lookup_timer = self.create_timer(10.0, self.update_current_topics_cb)

        self.current_topics = {}

        self.last_model_states_msg = None
        self.entity_topic_map = {}
        self.valid_entities = {}

        self.lock = Lock()

        self.get_logger().info("Motion Tracker Initialised")

    def update_current_topics_cb(self):
        self.current_topics = self.__get_current_vehicle_namespaces()

        model_prefixes = self.get_parameter('model_prefixes').value.split(' ')

        if self.last_model_states_msg is None:
            return 

        # Entity names are all <prefix>_<sys_id>

        self.lock.acquire()
        self.valid_entities = {}
        target_entity = {}
        for idx, n in enumerate(self.last_model_states_msg.name):
            if any([p in n for p in model_prefixes]):
                # print(f"Found {n} in {model_prefixes}")
                target = int(n.split('_')[1])
                if target in target_entity:
                    self.get_logger().warn(f"Entity {n} target id {target} is not unique, already collected: {target_entity}")
                else:
                    self.valid_entities[n] = idx
                    target_entity[target] = n

        # Find mapping between entity name and ros2 vehicle namespace
        self.entity_topic_map = {}
        for vname in self.current_topics:
            id = int(vname.split('_')[1])
            if id in target_entity:
                topic = f'/{vname}/{self.pose_topic}'
                self.entity_topic_map[vname] = (target_entity[id], self.create_publisher(PoseStamped, topic, 10))
            else:
                self.get_logger().warn(f"Vehicle {vname} with id {id} has no corresponding entity: {target_entity}")
        self.lock.release()

    def model_states_cb(self, msg):
        # https://github.com/ros-simulation/gazebo_ros_pkgs/blob/noetic-devel/gazebo_msgs/msg/ModelStates.msg
        stamp = self.get_clock().now()

        self.last_model_states_msg = msg

        self.lock.acquire()
        for vehicle_name, (entity_name, pose_pub) in self.entity_topic_map.items():
            pmsg = PoseStamped()
            pmsg.header.stamp = stamp.to_msg()
            pmsg.header.frame_id = self.frame_id
            pmsg.pose = msg.pose[self.valid_entities[entity_name]]
            pose_pub.publish(pmsg)

        self.lock.release()

            # self.get_logger().info(f'Pose sent from {entity_name} to {vehicle_name} on {topic}')

    def __get_current_vehicle_namespaces(self):
        topic_list = self.get_topic_names_and_types()
        namespaces = set()
        # self.get_logger().info('Found the following topics:')
        for topic_name, _ in topic_list:
            # self.get_logger().info(topic_name)
            if 'mavros' in topic_name:
                name = topic_name.split('/')[1]
                if name == 'mavros':
                    name = ''
                namespaces.add(name)
        # self.get_logger().info(f'Found {len(namespaces)} namespaces: {",".join(namespaces)}')
        return namespaces

def main(args=None):
    rclpy.init(args=args)
    mon = Monitor()
    rclpy.spin(mon)
    mon.destroy_node()
    rclpy.shutdown()