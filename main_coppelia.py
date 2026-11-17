import argparse
import csv
import math
import os
import sys

import numpy as np

import visualizacao as vis
from config import ASTAR, CENA, DWA, MAPA, NAV, ROBO
from navegador import NavegadorHibrido
from planejador_astar import mapa_de_imagem

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")


def ler_obstaculos(sim, sensores, alcance):
    pts = []
    for h in sensores:
        res, dist, ponto, _, _ = sim.readProximitySensor(h)
        if res > 0 and dist < alcance:
            pw = sim.multiplyVector(sim.getObjectMatrix(h, -1), ponto)
            pts.append((pw[0], pw[1]))
    return np.array(pts, dtype=float).reshape(-1, 2)


def detectar_orientacao(rgb, cpos, ortho, alvo):
    verm = (rgb[:, :, 0] > 180) & (rgb[:, :, 1] < 80) & (rgb[:, :, 2] < 80)
    if verm.sum() < 5:
        return None
    resultados = []
    for trocar in (False, True):
        v = verm.T if trocar else verm
        linhas, colunas = np.nonzero(v)
        res_y, res_x = v.shape
        px = ortho / max(res_x, res_y)
        mx, my = res_x * px / 2, res_y * px / 2
        c, r = (colunas.mean() + 0.5) * px, (linhas.mean() + 0.5) * px
        for fx in (True, False):
            for fy in (True, False):
                wx = cpos[0] + mx - c if fx else cpos[0] - mx + c
                wy = cpos[1] + my - r if fy else cpos[1] - my + r
                resultados.append((math.hypot(wx - alvo[0], wy - alvo[1]), trocar, fx, fy))
    resultados.sort()
    if resultados[0][0] < 0.35 and resultados[1][0] > resultados[0][0] + 0.3:
        return resultados[0][1:]
    return None


def mover_cinematico(sim, robo, pos, yaw, v, w, dt):
    th = yaw + ROBO.offset_theta
    nova = [pos[0] + v * math.cos(th) * dt, pos[1] + v * math.sin(th) * dt, pos[2]]
    ori = [0.0, 0.0, yaw + w * dt]
    try:
        sim.setObjectPosition(robo, nova, sim.handle_world)
        sim.setObjectOrientation(robo, ori, sim.handle_world)
    except Exception:
        sim.setObjectPosition(robo, sim.handle_world, nova)
        sim.setObjectOrientation(robo, sim.handle_world, ori)


def capturar_mapa(sim, cam, inicio, alvo, saida, mostrar):
    img, res = sim.getVisionSensorImg(cam)
    res_x, res_y = res[0], res[1]
    rgb = np.frombuffer(img, dtype=np.uint8).reshape(res_y, res_x, 3)
    cinza = rgb.mean(axis=2)
    binario = cinza > MAPA.limiar_binario
    vis.imagens_camera(cinza, binario, os.path.join(saida, "01_camera.png"), mostrar)

    ortho = sim.getObjectFloatParam(cam, sim.visionfloatparam_ortho_size)
    cpos = sim.getObjectPosition(cam, -1)
    orient = detectar_orientacao(rgb, cpos, ortho, alvo)
    if orient is None:
        print("Orientacao da camera: marcador vermelho do alvo nao encontrado; "
              "usando cam_inverter_x/y de config.py")
    else:
        trocar, MAPA.cam_inverter_x, MAPA.cam_inverter_y = orient
        if trocar:
            binario = binario.T
        print(f"Orientacao da camera detectada pelo marcador do alvo: "
              f"trocar_eixos={trocar}, inverter_x={MAPA.cam_inverter_x}, "
              f"inverter_y={MAPA.cam_inverter_y}")
    mapa, (wx, wy) = mapa_de_imagem(
        binario, cpos[0], cpos[1], ortho, MAPA.resolucao_min, ROBO.raio_inflar,
        MAPA.cam_inverter_x, MAPA.cam_inverter_y,
        limpar=[(inicio[0], inicio[1], MAPA.raio_limpeza_inicio),
                (alvo[0], alvo[1], MAPA.raio_limpeza_alvo)],
        raio_densidade=ASTAR.raio_densidade)
    print(f"Mapa: {mapa.nx}x{mapa.ny} celulas de {mapa.res:.3f} m | "
          f"{int(mapa.bruto.sum())} celulas ocupadas | camera em ({cpos[0]:.2f}, {cpos[1]:.2f}), "
          f"ortho={ortho:.2f} m")
    return mapa


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mostrar", action="store_true", help="abre as janelas dos graficos")
    ap.add_argument("--saida", default="resultados", help="pasta dos arquivos gerados")
    ap.add_argument("--vmax", type=float, default=None, help="substitui ROBO.v_max [m/s]")
    ap.add_argument("--cinematico", action="store_true",
                    help="move o corpo pela cinematica (sem fisica); use apos tornar_cinematico.py")
    args = ap.parse_args()
    os.makedirs(args.saida, exist_ok=True)
    vis.preparar(args.mostrar)
    if args.vmax:
        ROBO.v_max = args.vmax
    if args.cinematico:
        ROBO.offset_theta = 0.0

    from coppeliasim_zmqremoteapi_client import RemoteAPIClient
    print("Conectando ao CoppeliaSim...")
    client = RemoteAPIClient()
    sim = client.require("sim")

    robo = sim.getObject(CENA["robo"])
    m_esq = sim.getObject(CENA["motor_esq"])
    m_dir = sim.getObject(CENA["motor_dir"])
    sensores = [sim.getObject(n) for n in CENA["sensores"]]
    h_alvo = sim.getObject(CENA["alvo"])
    cam = sim.getObject(CENA["camera"])

    R, L = ROBO.raio_roda, ROBO.dist_rodas

    def aplicar(v, w):
        w = ROBO.sinal_w * w
        sim.setJointTargetVelocity(m_esq, (v - w * L / 2.0) / R)
        sim.setJointTargetVelocity(m_dir, (v + w * L / 2.0) / R)

    client.setStepping(True)
    sim.startSimulation()
    traj, linhas = [], []
    nav = None
    try:
        for _ in range(10):
            client.step()
        p0 = sim.getObjectPosition(robo, -1)
        pg = sim.getObjectPosition(h_alvo, -1)
        inicio, alvo = (p0[0], p0[1]), (pg[0], pg[1])

        mapa = capturar_mapa(sim, cam, inicio, alvo, args.saida, args.mostrar)
        nav = NavegadorHibrido(mapa, ROBO, ASTAR, DWA, NAV)
        if not nav.planejar(inicio, alvo):
            print("[ERRO] O A* nao encontrou caminho. Veja resultados/01_camera.png: "
                  "o mapa esta orientado certo? inicio/alvo estao dentro da area da camera?")
            return 2
        print(f"A*: {nav.astar.expandidos} nos expandidos | {len(nav.caminho_bruto)} celulas "
              f"-> {len(nav.chaves)} pontos-chave")
        vis.mapa_e_rota(nav, inicio, alvo, os.path.join(args.saida, "02_planejamento.png"),
                        inverter_eixos=MAPA.plot_inverter_eixos, titulo="Planejamento global (A* melhorado)",
                        mostrar=args.mostrar)

        v = w = 0.0
        dt = sim.getSimulationTimeStep()
        proximo_print, chegou = 0.0, False
        n_log = 0
        while True:
            t = sim.getSimulationTime()
            if t > NAV.tempo_max:
                print("[AVISO] Tempo maximo de simulacao atingido.")
                break
            pos = sim.getObjectPosition(robo, -1)
            yaw = sim.getObjectOrientation(robo, -1)[2]
            theta = yaw + ROBO.offset_theta
            obs = ler_obstaculos(sim, sensores, ROBO.alcance_sensor)

            v, w, status = nav.passo((pos[0], pos[1], theta), v, w, dt, t, obs)
            if status == "CHEGOU":
                chegou = True
                print(f"\n[SUCESSO] Alvo alcancado em t = {t:.1f} s")
                break
            if args.cinematico:
                mover_cinematico(sim, robo, pos, yaw, v, w, dt)
            else:
                aplicar(v, w)
            traj.append((pos[0], pos[1]))
            linhas.append((round(t, 3), pos[0], pos[1], theta, v, w, status, len(obs)))

            while n_log < len(nav.log):
                print(f"  [t={nav.log[n_log][0]:6.1f}s] {nav.log[n_log][1]}")
                n_log += 1
            if t >= proximo_print:
                d = math.hypot(nav.chaves[-1][0] - pos[0], nav.chaves[-1][1] - pos[1])
                print(f"[{status:<11}] t={t:6.1f}s | alvo a {d:.2f} m | v={v:.3f} w={w:+.2f} "
                      f"| obstaculos: {len(obs)}/5")
                proximo_print = t + 2.0
            client.step()
    except KeyboardInterrupt:
        print("\nInterrompido pelo usuario.")
        chegou = False
    finally:
        aplicar(0.0, 0.0)
        sim.stopSimulation()
        print("Simulacao encerrada.")

    if nav is not None and traj:
        traj = np.array(traj)
        dist = float(np.hypot(*np.diff(traj, axis=0).T).sum()) if len(traj) > 1 else 0.0
        tempo = linhas[-1][0]
        print("-" * 60)
        print(f"Distancia percorrida: {dist:.2f} m | tempo: {tempo:.1f} s | "
              f"v media: {dist / max(tempo, 1e-9):.3f} m/s | replanejamentos: {nav.replans}")
        with open(os.path.join(args.saida, "trajetoria.csv"), "w", newline="") as f:
            wr = csv.writer(f)
            wr.writerow(["t", "x", "y", "theta", "v", "w", "status", "n_obstaculos"])
            wr.writerows(linhas)
        vis.mapa_e_rota(nav, inicio, alvo, os.path.join(args.saida, "03_trajetoria_real.png"),
                        trajetoria=traj, inverter_eixos=MAPA.plot_inverter_eixos,
                        titulo="Trajetoria real - A* melhorado + DWA melhorado", mostrar=args.mostrar)
        print(f"Arquivos em: {os.path.abspath(args.saida)}")
    return 0 if chegou else 1


if __name__ == "__main__":
    sys.exit(main())
