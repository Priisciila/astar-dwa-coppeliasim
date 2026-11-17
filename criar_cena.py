import argparse
import math
import os
import sys

import numpy as np

from config import CENA, ROBO

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

Z0 = 0.002
R_RODA, L_EIXO = ROBO.raio_roda, ROBO.dist_rodas
BASE = (0.26, 0.20, 0.04)
Z_EIXO = Z0 + R_RODA
Z_BASE = Z_EIXO + 0.01
INICIO = (0.6, 0.6)
ALVO = (5.4, 5.0)
CAM_CENTRO, CAM_ALTURA, ORTHO, RES_IMG = (3.0, 3.0), 4.0, 6.0, 512
OBSTACULOS = {
    "Parede1": (1.8, 0.0, 2.1, 3.6),
    "Parede2": (3.6, 2.4, 3.9, 6.0),
    "Caixa": (4.5, 0.9, 5.0, 1.5),
}
ALT_OBST = 0.30
SENSORES = [
    ("SENSOR_ESQUERDO", 60, 0.07, 0.10),
    ("SENSOR_DIAG_ESQUERDO", 30, 0.12, 0.06),
    ("SENSOR_MEIO", 0, 0.13, 0.0),
    ("SENSOR_DIAG_DIREITO", -30, 0.12, -0.06),
    ("SENSOR_DIREITO", -60, 0.07, -0.10),
]

avisos = []


def tentar(rotulo, fn):
    try:
        return fn()
    except Exception as e:
        avisos.append(f"{rotulo}: {str(e).splitlines()[0] if str(e) else type(e).__name__}")
        return None


def const(sim, *nomes):
    for n in nomes:
        if hasattr(sim, n):
            return getattr(sim, n)
    raise AttributeError("nenhuma destas constantes existe nesta versao: " + ", ".join(nomes))


def poser(sim, h, pos=None, ori=None):
    mundo = sim.handle_world
    if pos is not None:
        try:
            sim.setObjectPosition(h, list(pos), mundo)
        except Exception:
            sim.setObjectPosition(h, mundo, list(pos))
    if ori is not None:
        try:
            sim.setObjectOrientation(h, list(ori), mundo)
        except Exception:
            sim.setObjectOrientation(h, mundo, list(ori))


def nomear(sim, h, nome):
    sim.setObjectAlias(h, nome)


def colorir(sim, h, rgb, emissao=None):
    sim.setShapeColor(h, None, sim.colorcomponent_ambient_diffuse, list(rgb))
    sim.setShapeColor(h, None, sim.colorcomponent_specular, [0.0, 0.0, 0.0])
    if emissao is not None:
        sim.setShapeColor(h, None, sim.colorcomponent_emission, list(emissao))


def forma(sim, tipo, dims, nome, pos, ori=(0, 0, 0), estatica=False, respondable=True,
          detectavel=True):
    h = sim.createPrimitiveShape(tipo, list(dims), 0)
    nomear(sim, h, nome)
    poser(sim, h, pos, ori)
    tentar(f"{nome}: estatico", lambda: sim.setObjectInt32Param(h, sim.shapeintparam_static, int(estatica)))
    tentar(f"{nome}: respondable",
           lambda: sim.setObjectInt32Param(h, sim.shapeintparam_respondable, int(respondable)))
    if not detectavel:
        tentar(f"{nome}: nao detectavel", lambda: sim.setObjectSpecialProperty(h, 0))
    return h


def pai(sim, filho, pai_):
    sim.setObjectParent(filho, pai_, True)


def apagar_existentes(sim):
    nomes = ["/" + CENA["robo"].strip("/"), CENA["alvo"], CENA["camera"], "/PisoEscuro"] + \
            ["/" + n for n in OBSTACULOS]
    for n in nomes:
        try:
            h = sim.getObject(n)
        except Exception:
            continue
        try:
            sim.removeObjects(sim.getObjectsInTree(h))
        except Exception:
            sim.removeObject(h)
        print(f"  removido: {n}")


def atrito_zero(sim, h, nome):
    ok = False
    for motor in ("bullet", "ode", "newton", "vortex", "mujoco"):
        for suf in ("body_friction", "body_dynamicfriction", "joint_friction"):
            c = getattr(sim, f"{motor}_{suf}", None)
            if c is None or suf == "joint_friction":
                continue
            try:
                sim.setEngineFloatParam(c, h, 0.0)
                ok = True
            except Exception:
                pass
    if not ok:
        avisos.append(f"{nome}: nao consegui zerar o atrito (o robo pode girar com dificuldade)")


def construir(sim):
    prim_cub = const(sim, "primitiveshape_cuboid")
    prim_cil = const(sim, "primitiveshape_cylinder")
    prim_esf = const(sim, "primitiveshape_spheroid", "primitiveshape_sphere")

    piso = forma(sim, prim_cub, (8.0, 8.0, 0.02), "PisoEscuro", (3.0, 3.0, Z0 - 0.01), estatica=True)
    colorir(sim, piso, (0.04, 0.04, 0.04))

    for nome, (x0, y0, x1, y1) in OBSTACULOS.items():
        h = forma(sim, prim_cub, (x1 - x0, y1 - y0, ALT_OBST), nome,
                  ((x0 + x1) / 2, (y0 + y1) / 2, Z0 + ALT_OBST / 2), estatica=True)
        colorir(sim, h, (1, 1, 1), emissao=(1, 1, 1))

    alvo = sim.createDummy(0.08)
    nomear(sim, alvo, CENA["alvo"].strip("/"))
    poser(sim, alvo, (ALVO[0], ALVO[1], Z0 + 0.05))
    marc = forma(sim, prim_cil, (0.24, 0.24, 0.004), "marcador", (ALVO[0], ALVO[1], Z0 + 0.002),
                 estatica=True, respondable=False, detectavel=False)
    colorir(sim, marc, (1, 0, 0), emissao=(1, 0, 0))
    pai(sim, marc, alvo)

    base = forma(sim, prim_cub, BASE, CENA["robo"].strip("/"), (INICIO[0], INICIO[1], Z_BASE),
                 detectavel=False)
    colorir(sim, base, (0.1, 0.25, 0.9))

    for nome, dx in (("rodaboba_frente", 0.10), ("rodaboba_tras", -0.10)):
        c = forma(sim, prim_esf, (0.052, 0.052, 0.052), nome,
                  (INICIO[0] + dx, INICIO[1], Z0 + 0.026 + 0.004), detectavel=False)
        colorir(sim, c, (0.7, 0.7, 0.7))
        atrito_zero(sim, c, nome)
        pai(sim, c, base)

    tipo_j = const(sim, "joint_revolute_subtype", "joint_revolute")
    modo_j = const(sim, "jointmode_dynamic", "jointmode_force")
    eixo_y = (-math.pi / 2, 0.0, 0.0)
    for nome, lado in ((CENA["motor_esq"].split("/")[-1], +1), (CENA["motor_dir"].split("/")[-1], -1)):
        py = INICIO[1] + lado * L_EIXO / 2
        j = sim.createJoint(tipo_j, modo_j, 0, [0.03, 0.01])
        nomear(sim, j, nome)
        poser(sim, j, (INICIO[0], py, Z_EIXO), eixo_y)
        roda = forma(sim, prim_cil, (2 * R_RODA, 2 * R_RODA, 0.02), "roda_" + nome,
                     (INICIO[0], py, Z_EIXO), eixo_y, detectavel=False)
        colorir(sim, roda, (0.05, 0.05, 0.05))
        pai(sim, j, base)
        pai(sim, roda, j)
        tentar(f"{nome}: ciclico (sem limite de angulo)",
               lambda j=j: sim.setJointInterval(j, True, [-math.pi, math.pi]))
        tentar(f"{nome}: forca maxima", lambda j=j: sim.setJointTargetForce(j, 1.0))
        tentar(f"{nome}: modo velocidade (dynctrl)", lambda j=j: sim.setObjectInt32Param(
            j, sim.jointintparam_dynctrlmode, sim.jointdynctrl_velocity))
        tentar(f"{nome}: motor ligado", lambda j=j: sim.setObjectInt32Param(
            j, sim.jointintparam_motor_enabled, 1))
        tentar(f"{nome}: controle de posicao desligado", lambda j=j: sim.setObjectInt32Param(
            j, sim.jointintparam_ctrl_enabled, 0))

    sub_ray = const(sim, "proximitysensor_ray_subtype")
    for nome, ang, sx, sy in SENSORES:
        s = tentar(f"{nome}: criar", lambda: sim.createProximitySensor(
            sub_ray, 16, 0, [1, 1, 1, 1, 1, 1, 0, 0],
            [0.005, ROBO.alcance_sensor, 0.01, 0.01, 0.01, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
             0.0, 0.0]))
        if s is None:
            continue
        nomear(sim, s, nome)
        poser(sim, s, (INICIO[0] + sx, INICIO[1] + sy, Z_BASE), (0.0, math.pi / 2, math.radians(ang)))
        pai(sim, s, base)

    cam = sim.createVisionSensor(0, [RES_IMG, RES_IMG, 0, 0],
                                 [0.01, 10.0, ORTHO, 0.2, 0.2, 0.4, 0, 0, 0, 0, 0])
    nomear(sim, cam, CENA["camera"].strip("/"))
    poser(sim, cam, (CAM_CENTRO[0], CAM_CENTRO[1], CAM_ALTURA), (math.pi, 0.0, 0.0))
    tentar("camera: tamanho ortografico",
           lambda: sim.setObjectFloatParam(cam, sim.visionfloatparam_ortho_size, ORTHO))
    return base


def autoteste(sim, client):
    print("\nAutoteste (simulacao de ~3 s)...")
    client.setStepping(True)
    sim.startSimulation()
    resumo = {}
    try:
        for _ in range(10):
            client.step()
        robo = sim.getObject(CENA["robo"])
        m_esq, m_dir = sim.getObject(CENA["motor_esq"]), sim.getObject(CENA["motor_dir"])
        p0 = sim.getObjectPosition(robo, -1)
        yaw0 = sim.getObjectOrientation(robo, -1)[2]
        v = 0.10 / R_RODA
        sim.setJointTargetVelocity(m_esq, v)
        sim.setJointTargetVelocity(m_dir, v)
        t_fim = sim.getSimulationTime() + 1.5
        while sim.getSimulationTime() < t_fim:
            client.step()
        sim.setJointTargetVelocity(m_esq, 0.0)
        sim.setJointTargetVelocity(m_dir, 0.0)
        p1 = sim.getObjectPosition(robo, -1)
        desloc = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
        resumo["deslocamento_m"] = desloc
        resumo["sentido_graus"] = math.degrees(math.atan2(p1[1] - p0[1], p1[0] - p0[0]) - yaw0)

        cam = sim.getObject(CENA["camera"])
        img, res = sim.getVisionSensorImg(cam)
        rgb = np.frombuffer(img, dtype=np.uint8).reshape(res[1], res[0], 3)
        resumo["pixels_brancos"] = int((rgb.mean(axis=2) > 245).sum())
        resumo["pixels_vermelhos"] = int(((rgb[:, :, 0] > 180) & (rgb[:, :, 1] < 80)
                                          & (rgb[:, :, 2] < 80)).sum())
    finally:
        sim.stopSimulation()
    return resumo


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--recriar", action="store_true", help="apaga objetos existentes com esses nomes")
    ap.add_argument("--saida", default="cena_robo_astar_dwa.ttt")
    ap.add_argument("--sem-autoteste", action="store_true")
    args = ap.parse_args()

    from coppeliasim_zmqremoteapi_client import RemoteAPIClient
    client = RemoteAPIClient()
    sim = client.require("sim")

    if sim.getSimulationState() != sim.simulation_stopped:
        sim.stopSimulation()
        while sim.getSimulationState() != sim.simulation_stopped:
            pass
    try:
        sim.getObject(CENA["robo"])
        existe = True
    except Exception:
        existe = False
    if existe and not args.recriar:
        print(f"[ERRO] A cena aberta ja tem {CENA['robo']}. Use --recriar para apagar e recriar.")
        return 2
    if args.recriar:
        apagar_existentes(sim)

    print("Montando a cena...")
    construir(sim)
    print(f"  {len(sim.getObjectsInTree(sim.handle_scene))} objetos na cena.")

    resumo = {} if args.sem_autoteste else autoteste(sim, client)

    caminho = os.path.abspath(args.saida)
    tentar("salvar cena", lambda: sim.saveScene(caminho))
    print("\n" + "=" * 60)
    if resumo:
        d = resumo["deslocamento_m"]
        print(f"Motores : andou {d:.3f} m (esperado ~0.15 m) "
              f"{'OK' if d > 0.05 else '-> NAO ANDOU, veja os avisos'}")
        print(f"Frente  : o robo andou a {resumo['sentido_graus']:+.0f} graus do eixo yaw "
              "(rode calibrar_robo.py --aplicar para gravar no config.py)")
        print(f"Camera  : {resumo['pixels_brancos']} px brancos (obstaculos), "
              f"{resumo['pixels_vermelhos']} px vermelhos (marcador do alvo) "
              f"{'OK' if resumo['pixels_brancos'] > 500 and resumo['pixels_vermelhos'] > 20 else '-> CONFIRA A CAMERA'}")
    if avisos:
        print("\nAVISOS (passos que falharam; a cena pode precisar de ajuste manual):")
        for a in avisos:
            print("  -", a)
    print(f"\nCena salva em: {caminho}")
    print("Proximo passo: python calibrar_robo.py --aplicar   e depois   python main_coppelia.py")
    return 0


if __name__ == "__main__":
    sys.exit(main())
