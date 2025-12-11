#!/usr/bin/env python
import rclpy

from tiago_lfc.tools.model_utils import load_reduced_pinocchio_model

def main():
    rclpy.init()

    arm_joints = [
        'arm_1_joint', 'arm_2_joint', 'arm_3_joint',
        'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint'
    ]

    try:
        print("Start testing model loading utility...")
        model, data = load_reduced_pinocchio_model(arm_joints)
        
        print("\n--- VVerification ---")
        print(f"Model created successfully!")
        print(f"Number of DOF (nq) : {model.nq}")
        
        # Visual check of joint names
        print("Names of preserved joints:")
        for name in model.names:
            if name != "universe":
                print(f" - {name}")

    except Exception as e:
        print(f"ERROR: {e}")

    rclpy.shutdown()

if __name__ == '__main__':
    main()
