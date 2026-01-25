#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import zmq
import json
import numpy as np

# --- KONFIGURÁCIÓ ---
LEADER_IP = "192.168.1.100"  # <--- Írd át a LEADER GÉP IP címére!
ZMQ_PORT = 5555

class ZMQToROS2Bridge(Node):
    def __init__(self):
        super().__init__('zmq_ros2_bridge')
        
        # 1. ROS2 Publisher létrehozása
        # Az AR4 driverétől függ, milyen topicot vár. 
        # Általában: '/joint_group_position_controller/commands' vagy hasonló
        self.publisher_ = self.create_publisher(Float64MultiArray, '/ar4/commands', 10)
        
        # Opcionális: JointState publisher vizualizációhoz (RViz)
        self.state_pub_ = self.create_publisher(JointState, '/joint_states_target', 10)

        # 2. ZMQ Subscriber beállítása
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.setsockopt(zmq.CONFLATE, 1) # Csak a legfrissebb adat!
        
        # Csatlakozás a Leader géphez
        address = f"tcp://{LEADER_IP}:{ZMQ_PORT}"
        self.get_logger().info(f'Csatlakozás a Leaderhez: {address}')
        self.socket.connect(address)
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "")

        # 3. Időzítő (Timer) a folyamatos olvasáshoz (pl. 50Hz)
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.get_logger().info('Híd elindult! 🌉')

    def timer_callback(self):
        try:
            # Nem blokkoló olvasás
            msg_bytes = self.socket.recv(flags=zmq.NOBLOCK)
            data = json.loads(msg_bytes)
            
            # --- ADAT FELDOLGOZÁSA ---
            # A Leader 5 vagy 6 tengelyt küld. Az AR4-nek 6 kell.
            leader_joints = data['joints'] # [j1, j2, j3, j4, j5]
            gripper = data['gripper']
            
            # Mapping: Itt kell megfeleltetni a Leader motorjait az AR4 motorjainak
            # Ha a Leadernek csak 5 motorja van, a 6. (J6) legyen 0.0 vagy a gripper állása
            
            # Példa mapping (SO100 -> AR4):
            ar4_joints = [
                leader_joints[0], # J1
                leader_joints[1], # J2
                leader_joints[2], # J3
                leader_joints[3], # J4
                leader_joints[4], # J5
                0.0               # J6 (ha nincs a Leaderen J6 forgatás)
            ]

            # --- ROS2 ÜZENET KÜLDÉSE (Command) ---
            ros_msg = Float64MultiArray()
            ros_msg.data = ar4_joints
            self.publisher_.publish(ros_msg)

            # --- ROS2 ÜZENET KÜLDÉSE (JointState - Opcionális) ---
            js_msg = JointState()
            js_msg.header.stamp = self.get_clock().now().to_msg()
            js_msg.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
            js_msg.position = ar4_joints
            self.state_pub_.publish(js_msg)

            # Log (csak néha)
            # self.get_logger().info(f'Publikálva: {ar4_joints}')

        except zmq.Again:
            pass # Nincs új üzenet, nem baj
        except Exception as e:
            self.get_logger().error(f'Hiba: {e}')

def main(args=None):
    rclpy.init(args=args)
    bridge_node = ZMQToROS2Bridge()
    
    try:
        rclpy.spin(bridge_node)
    except KeyboardInterrupt:
        pass
    finally:
        bridge_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()