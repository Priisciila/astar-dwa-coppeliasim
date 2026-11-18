"""Converte o robo da cena em modo CINEMATICO (sem fisica) e salva cena_cinematica.ttt.

Todas as pecas do /Cuboid viram estaticas e nao-respondable; os joints viram cinematicos;
a pose inicial e restaurada (0,6; 0,6; yaw 0). O main_coppelia.py --cinematico move o corpo
pela cinematica diferencial. Sensores de proximidade, camera e alvo seguem normais.

Uso:  python tornar_cinematico.py
"""
import math
import os
import sys

from config import CENA

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

INICIO = (0.6, 0.6)
Z_BASE = 0.002 + 0.036 + 0.01


def main():
    from coppeliasim_zmqremoteapi_client import RemoteAPIClient
    sim = RemoteAPIClient().require("sim")
    if sim.getSimulationState() != sim.simulation_stopped:
        sim.stopSimulation()
        while sim.getSimulationState() != sim.simulation_stopped:
            pass
    robo = sim.getObject(CENA["robo"])
    n_forma = n_junta = 0
    for h in sim.getObjectsInTree(robo):
        tipo = sim.getObjectType(h)
        if tipo == sim.sceneobject_shape:
            sim.setObjectInt32Param(h, sim.shapeintparam_static, 1)
            sim.setObjectInt32Param(h, sim.shapeintparam_respondable, 0)
            n_forma += 1
        elif tipo == sim.sceneobject_joint:
            modo = getattr(sim, "jointmode_kinematic", None)
            if modo is None:
                modo = sim.jointmode_passive
            sim.setJointMode(h, modo, 0)
            n_junta += 1
    pos, ori = [INICIO[0], INICIO[1], Z_BASE], [0.0, 0.0, 0.0]
    try:
        sim.setObjectPosition(robo, pos, sim.handle_world)
        sim.setObjectOrientation(robo, ori, sim.handle_world)
    except Exception:  # noqa: BLE001
        sim.setObjectPosition(robo, sim.handle_world, pos)
        sim.setObjectOrientation(robo, sim.handle_world, ori)
    print(f"{n_forma} pecas -> estaticas | {n_junta} joints -> cinematicos | pose inicial restaurada")

    caminho = os.path.abspath("cena_cinematica.ttt")
    sim.saveScene(caminho)
    print(f"Cena salva em {caminho}")
    print("Agora rode:  python main_coppelia.py --cinematico")


if __name__ == "__main__":
    main()
