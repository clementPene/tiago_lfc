'''
LFC Bridge Example: Simple PD Control to a Static Target

Description:
    This node serves as a "Bridge" between Python logic and the low-level
    'linear_feedback_controller' (LFC) running in C++.
    It implements a simple PD controller that holds the robot at a specific
    'SAFE_POS' configuration.

Communication Architecture:
    1.  Topic: /control (Type: linear_feedback_controller_msgs/msg/Control)
        - This node publishes to /control at 100Hz.
        - The message contains:
            a. Feedforward torque (u_ff)
            b. Feedback Gain Matrix (K) of size (nu, 2*nx)
            c. Reference State (x_ref), encapsulated in a Sensor message.

    2.  Control Law (computed on C++ side):
        u = u_ff - K * (x_measured - x_ref)
        
        Where x = [position; velocity].
        By setting inputs correctly, this acts as:
        u = u_ff + Kp * (pos_ref - pos_meas) + Kd * (vel_ref - vel_meas)

Data Conversion (Crucial Step):
    Standard ROS 2 Float64MultiArray messages usually require manual population 
    of the 'layout' field (dimensions, strides) for C++ matrix libraries (like Eigen) 
    to read them correctly. 
    
    To avoid segmentation faults (Segfaults), we use the library:
    pool 'linear_feedback_controller_msgs_py'.
    
    Specifically, 'control_numpy_to_msg()' handles the conversion from 
    standard Numpy arrays to the strict memory layout required by the LFC.
'''

import rclpy
from rclpy.node import Node
import numpy as np

from linear_feedback_controller_msgs.msg import Control

# these are helpers found in learning_linear_feedback_controller package, 
# helping to transcribe numpy arrays to ROS messages
from linear_feedback_controller_msgs_py.numpy_conversions import control_numpy_to_msg
import linear_feedback_controller_msgs_py.lfc_py_types as lfc_py_types

# these are the moving joint names, and they have to be the same as the ones in lfc param file
JOINT_NAMES = [
    'arm_1_joint', 'arm_2_joint', 'arm_3_joint',
    'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint'
]
NV = len(JOINT_NAMES)

# Position you want the robot to reach. If you chose impossible configuration, gazebo will crash.
# chose with care!
SAFE_POS = np.array([
    1.50,  # arm_1 (shoudler)
    0.0,  # arm_2  (shoudler)
    0.0,   # arm_3 (elbow)
    0.0,   # arm_4 (elbow)
    0.0,   # arm_5 
    0.0,   # arm_6
    0.0    # arm_7 (wrist)
])

class LFCBridge(Node):
    def __init__(self):
        super().__init__('lfc_bridge')
        
        # Publisher to the control command
        self.publisher_ = self.create_publisher(Control, '/control', 1)

        # Control loop timer running at 100 Hz (10ms dt)
        self.dt = 0.01 
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info("LFC Bridge Active. Moving towards SAFE_POS...")

    def control_loop(self):
        # Define Gains (Simple PD Controller)
        # Kp: Stiffness (Proportional gain)
        # Kd: Damping (Derivative gain)
        Kp = 200.0
        Kd = 15.0

        # The Feedback Gain Matrix K has shape (nu, 2*nu) -> (7, 14) ideally.
        # - The first NV columns correspond to Position errors.
        # - The last NV columns correspond to Velocity errors.
        K = np.zeros((NV, 2*NV))
        K[:, :NV] = np.eye(NV) * Kp
        K[:, NV:] = np.eye(NV) * Kd
        
        # Build Python Object (LFC Types)
        # The controller applies: u = feedforward - K * (x_current - x_ref)
        # To make the robot go to SAFE_POS, we set SAFE_POS as the reference "x_ref" (initial_state).
        
        sensor_obj = lfc_py_types.Sensor(
            base_pose=np.zeros(7),
            base_twist=np.zeros(6),
            joint_state=lfc_py_types.JointState(
                name=JOINT_NAMES,
                position=SAFE_POS,
                velocity=np.zeros(NV),
                effort=np.zeros(NV)
            ),
            contacts=[]
        )

        # Feedforward Torque
        # Set to zero for now. TODO : I don't remember if gravity is compensated by LFC or not.
        tau_ff = np.zeros((NV, 1))

        # Create Control object
        ctrl_obj = lfc_py_types.Control(
            feedback_gain=K,
            feedforward=tau_ff,
            initial_state=sensor_obj
        )

        # ROS2 conversion helper from pool 'linear_feedback_controller_msgs_py'
        msg = control_numpy_to_msg(ctrl_obj)

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LFCBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
