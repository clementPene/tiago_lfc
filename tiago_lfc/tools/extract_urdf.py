#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
import sys
import numpy as np
import pinocchio as pin


class ModelLiveChecker(Node):
    def __init__(self):
        super().__init__('model_live_checker')

        # Liste des joints que tu veux contrôler (Copie de ton YAML)
        self.target_joints = [
            'arm_1_joint', 'arm_2_joint', 'arm_3_joint', 
            'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint'
        ]

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        
        self.create_subscription(
            String, 
            '/robot_description', 
            self.callback_urdf, 
            qos
        )
        print("⏳ En attente du robot_description...")

    def callback_urdf(self, msg):
        print("✅ URDF Reçu !")
        xml_content = msg.data

        print(f"🔄 Tentative de réduction pour garder uniquement : {len(self.target_joints)} joints...")
        
        try:
            model, data = create_reduced_model_from_xml(xml_content, self.target_joints)
            
            print("-" * 30)
            print(f"🎉 SUCCÈS !")
            print(f"Modèle réduit créé avec {model.nq} DDL (degrés de liberté).")
            print(f"Noms des joints dans le modèle Pinocchio :")
            for name in model.names:
                if name != "universe":
                    print(f" - {name}")
            print("-" * 30)

            if model.nq == 7:
                print("✅ C'est PARFAIT pour ton MPC (7 DoF).")
            else:
                print("⚠️  Attention, le nombre de DDL n'est pas 7.")

        except Exception as e:
            print(f"❌ Erreur lors de la réduction : {e}")

        # On quitte proprement une fois le test fini
        sys.exit(0)

def create_reduced_model_from_xml(urdf_xml_string, controlled_joint_names):
    """
    Entrée : 
        - urdf_xml_string : Le contenu du fichier URDF (string)
        - controlled_joint_names : Liste des joints qu'on veut garder (liste de strings)
    
    Sortie :
        - model : Le modèle Pinocchio réduit (taille 7 par exemple)
        - data : Les data associées
    """
    
    # 1. Construire le modèle COMPLET depuis le XML
    # C'est ici que Pinocchio parse tout (roues, tête, etc.)
    full_model = pin.buildModelFromXML(urdf_xml_string)
    
    # 2. Trouver les IDs des joints à BLOQUER (Lock)
    # La logique : "Tout ce qui n'est pas dans ma liste 'controlled', je le bloque"
    locked_joint_ids = []
    
    for name in full_model.names:
        if name == "universe": continue # On ne bloque pas l'univers !
        
        # Si le joint n'est PAS dans notre liste de commande, on l'ajoute à la liste noire
        if name not in controlled_joint_names:
            # On récupère l'ID Pinocchio de ce joint
            joint_id = full_model.getJointId(name)
            locked_joint_ids.append(joint_id)

    # 3. Créer le modèle RÉDUIT
    # On a besoin d'une config de ref (q0) pour savoir à quel angle figer les joints bloqués
    q_ref = np.zeros(full_model.nq) 
    
    # La magie opère ici : Pinocchio crée un nouveau modèle mathématique plus petit
    reduced_model = pin.buildReducedModel(full_model, locked_joint_ids, q_ref)
    reduced_data = reduced_model.createData()

    return reduced_model, reduced_data

def main():
    rclpy.init()
    node = ModelLiveChecker()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
