#!/usr/bin/env python
import rclpy
import numpy as np

# Tes imports
from tiago_lfc.tools.model_utils import load_reduced_pinocchio_model
from tiago_lfc.mpc_controller import MPCController

def main():
    rclpy.init()

    # Définition des joints du bras
    arm_joints = [
        'arm_1_joint', 'arm_2_joint', 'arm_3_joint',
        'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint'
    ]

    try:
        print("--- 1. Chargement du modèle ---")
        # On charge le modèle (la fonction renvoie modèle + données)
        returned_objects = load_reduced_pinocchio_model(arm_joints)
        
        # Gestion propre du retour (tuple ou objet seul)
        if isinstance(returned_objects, tuple):
            model = returned_objects[0]
        else:
            model = returned_objects

        print(f"Modèle chargé. nq = {model.nq}, nv = {model.nv}")

        print("\n--- 2. Initialisation du MPC ---")
        # On instancie le contrôleur avec un horizon de 10 pas et 10ms de dt
        mpc = MPCController(model, dt=0.01, T=10)
        
        print("\n--- 3. Simulation d'un pas de calcul ---")
        # Création de fausses données (état actuel vs cible)
        
        # État actuel : robot à zéro (tout droit)
        q_current = np.zeros(model.nq)
        v_current = np.zeros(model.nv)
        x_current = np.concatenate([q_current, v_current])
        
        # État cible : on veut bouger le premier joint à 0.5 rad
        q_ref = np.zeros(model.nq)
        q_ref[0] = 0.5 
        v_ref = np.zeros(model.nv)
        x_ref = np.concatenate([q_ref, v_ref])

        # Appel du solveur
        print(f"Cible demandée : Joint 1 = {q_ref[0]}")
        torques = mpc.solve(x_current, x_ref)

        print("\n--- RÉSULTAT ---")
        print(f"Couples calculés (u0) :\n{torques}")
        print(f"Taille du vecteur commande : {len(torques)}")
        
        if np.all(torques == 0):
            print("ATTENTION : Les couples sont tous à 0 (vérifier les poids ou l'état).")
        else:
            print("SUCCÈS : Le contrôleur génère des couples actifs.")

    except Exception as e:
        print(f"ERREUR : {e}")
        # On affiche la stack trace pour mieux comprendre
        import traceback
        traceback.print_exc()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
