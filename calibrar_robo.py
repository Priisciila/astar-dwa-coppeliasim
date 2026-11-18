import argparse
import math
import os
import re
import sys

from config import CENA, ROBO

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")


def norm(a):
    return math.atan2(math.sin(a), math.cos(a))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--aplicar', action='store_true', help='grava os valores em config.py')
    args = ap.parse_args()
    from coppeliasim_zmqremoteapi_client import RemoteAPIClient
    client = RemoteAPIClient()
    sim = client.require("sim")
    robo = sim.getObject(CENA["robo"])
    m_esq = sim.getObject(CENA["motor_esq"])
    m_dir = sim.getObject(CENA["motor_dir"])
    R, L = ROBO.raio_roda, ROBO.dist_rodas

    def rodar(v, w, dur):
        sim.setJointTargetVelocity(m_esq, (v - w * L / 2.0) / R)
        sim.setJointTargetVelocity(m_dir, (v + w * L / 2.0) / R)
        t_fim = sim.getSimulationTime() + dur
        while sim.getSimulationTime() < t_fim:
            client.step()
        sim.setJointTargetVelocity(m_esq, 0.0)
        sim.setJointTargetVelocity(m_dir, 0.0)
        for _ in range(10):
            client.step()

    client.setStepping(True)
    sim.startSimulation()
    try:
        for _ in range(10):
            client.step()
        p0, yaw0 = sim.getObjectPosition(robo, -1), sim.getObjectOrientation(robo, -1)[2]
        rodar(0.10, 0.0, 1.5)
        p1 = sim.getObjectPosition(robo, -1)
        dx, dy = p1[0] - p0[0], p1[1] - p0[1]
        desloc = math.hypot(dx, dy)
        print(f"\nReto: deslocou {desloc:.3f} m (esperado ~0.15 m)")
        offset = norm(math.atan2(dy, dx) - yaw0)
        sugerido = round(offset / (math.pi / 2)) * (math.pi / 2)
        sugerido = math.pi if abs(abs(sugerido) - math.pi) < 1e-9 else sugerido

        yaw_a = sim.getObjectOrientation(robo, -1)[2]
        rodar(0.0, 0.5, 1.5)
        dyaw = norm(sim.getObjectOrientation(robo, -1)[2] - yaw_a)
        print(f"Giro com w=+0.5: yaw variou {math.degrees(dyaw):+.1f} graus (esperado ~ +43)")
        sinal = 1.0 if dyaw > 0 else -1.0
    finally:
        sim.stopSimulation()

    print()
    print(f"offset_theta: float = {sugerido:.5f}   # medido: {offset:.3f} rad "
          f"({math.degrees(offset):.1f} graus); atual = {ROBO.offset_theta:.5f}")
    print(f"sinal_w: float = {sinal:+.1f}          # atual = {ROBO.sinal_w:+.1f}")
    if args.aplicar:
        caminho = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config.py")
        txt = open(caminho, encoding="utf-8").read()
        valor = "math.pi" if abs(abs(sugerido) - math.pi) < 1e-9 else f"{sugerido:.5f}"
        txt = re.sub(r"(offset_theta: float = )[^\s#]+", lambda m: m.group(1) + valor, txt)
        txt = re.sub(r"(sinal_w: float = )[-+0-9.]+", lambda m: m.group(1) + f"{sinal:+.1f}", txt)
        open(caminho, "w", encoding="utf-8").write(txt)
        print(f"\n[OK] config.py atualizado: offset_theta = {valor}, sinal_w = {sinal:+.1f}")
    if abs(norm(sugerido - offset)) > math.radians(20):
        print("[AVISO] o offset medido nao e multiplo de 90 graus; usado o mais proximo.")


if __name__ == "__main__":
    main()
