#!/usr/bin/env python
"""
Created on Mon Dec  2 17:03:34 2019

@author: mahmoud
"""

import rclpy
# from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import SpawnEntity

from geometry_msgs.msg import *
from rospkg import RosPack
from pedsim_msgs.msg  import AgentStates

from ament_index_python.packages import get_package_share_directory

# xml file containing a gazebo model to represent agent, currently is represented by a cubic but can be changed
global xml_file

def actor_poses_callback(actors):
    global node, xml_string

    for actor in actors.agent_states:
        actor_id = str( actor.id )
        actor_pose = actor.pose
        node.get_logger().info("Spawning model: actor_id = %s" % actor_id)

        model_pose = Pose(Point(x= actor_pose.position.x,
                               y= actor_pose.position.y,
                               z= actor_pose.position.z),
                         Quaternion(actor_pose.orientation.x,
                                    actor_pose.orientation.y,
                                    actor_pose.orientation.z,
                                    actor_pose.orientation.w) )

        # cli.spawn_model(actor_id, xml_string, "", model_pose, "world")
        spawn_entity(node, actor_id, xml_string, "", model_pose, "world")
    
    node.get_logger().info("all agents have been spawned !")
    node.destroy_node()
    rclpy.shutdown()


def spawn_entity(node, name, entity_xml, robot_namespace, initial_pose, reference_frame, gazebo_namespace="", timeout=DEFAULT_TIMEOUT):
    if timeout < 0:
        node.get_logger().error('spawn_entity timeout must be greater than zero')
        return False
    node.get_logger().info(
        'Waiting for service %s/spawn_entity, timeout = %.f' % (
            gazebo_namespace, timeout))
    node.get_logger().info('Waiting for service %s/spawn_entity' % gazebo_namespace)
    client = node.create_client(SpawnEntity, '%s/spawn_entity' % gazebo_namespace)
    if client.wait_for_service(timeout_sec=timeout):
        req = SpawnEntity.Request()
        req.name = name
        req.xml = str(entity_xml, 'utf-8')
        req.robot_namespace = robot_namespace
        req.initial_pose = initial_pose
        req.reference_frame = reference_frame
        node.get_logger().info('Calling service %s/spawn_entity' % gazebo_namespace)
        srv_call = client.call_async(req)
        while rclpy.ok():
            if srv_call.done():
                node.get_logger().info('Spawn status: %s' % srv_call.result().status_message)
                break
            rclpy.spin_once(node)
        return srv_call.result().success
    node.get_logger().error(
        'Service %s/spawn_entity unavailable. Was Gazebo started with GazeboRosFactory?')
    return False


def main(args=args):
    global node, xml_string
    rclpy.init()
    
    node = rclpy.create_node('spawn_pedsim_agents')
    cli = node.create_client(SpawnEntity, "/spawn_entity")

    # rospack1 = RosPack()
    # pkg_path = rospack1.get_path('pedsim_gazebo_plugin')
    # default_actor_model_file = pkg_path + "/models/actor_model.sdf"
    pkg_path = get_package_share_directory('pedsim_gazebo_plugin')
    default_actor_model_file = pkg_path + "models/actor_model.sdf"
    node.declare_parameter('~actor_model_file', default_actor_model_file)

    actor_model_file = node.get_parameter('~actor_model_file')
    file_xml = open(actor_model_file)
    xml_string = file_xml.read()


    node.get_logger().info("Waiting for gazebo services...")
    while not cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not availabe, waiting again...')
    
    node.get_logger().info("service: spawn_sdf_model is available ....")

    actor_poses_sub_ = node.create_subscription(
        AgentStates,
        "/pedsim_simulator/simulated_agents",
        actor_poses_callback,
        10
    )

    rclpy.spin(node)


if __name__ == '__main__':
    main()
