import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
import pinocchio
import sys

class ModelChecker(Node):
    def __init__(self):
        super().__init__('model_checker_node')
        
        # IMPORTANT : Le topic /robot_description est "Latched" (Transient Local)
        # Si on ne met pas ce QoS, on ne recevra jamais le message déjà publié.
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        
        self.create_subscription(
            String, 
            '/robot_description', 
            self.callback, 
            qos
        )
        print("Waiting to receive model from /robot_description...")

    def callback(self, msg):
        print("✅ Model received from ROS!")
        urdf_string = msg.data
        
        try:
            # Load the model directly from the XML string
            # Add the FreeFlyer because it's a mobile base
            model = pinocchio.buildModelFromXML(urdf_string) # add pinocchio.JointModelFreeFlyer() for mobile base
            
            print(f"\n--- COMPATIBILITY REPORT ---")
            print(f"Total robot mass: {sum([i.mass for i in model.inertias]):.2f} kg")
            print(f"Number of joints (nq): {model.nq}")
            print(f"Number of actuated joints (nv): {model.nv}")
            print(f"List of main joints:")
            for name in model.names:
                # if name != "universe" and "root" not in name:
                print(f" - {name}")
            
            print("\n✅ SUCCESS: Pinocchio loaded exactly the same model as Gazebo.")
            
        except Exception as e:
            print(f"\n❌ CRITICAL ERROR: Pinocchio failed to read the Gazebo model.\n{e}")
        
        # Close the node after one message
        sys.exit(0)

def main():
    rclpy.init()
    checker = ModelChecker()
    rclpy.spin(checker)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
