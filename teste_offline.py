import argparse
import math
import sys

import numpy as np

import visualizacao as vis
from config import ASTAR, DWA, MAPA, NAV, ROBO
from navegador import NavegadorHibrido
from planejador_astar import mapa_de_imagem

PAREDES = [(1.8, 0.0, 2.1, 3.6), (3.6, 2.4, 3.9, 6.0), (4.5, 0.9, 5.0, 1.5)]
CAIXA_OCULTA = (2.55, 2.75, 3.05, 3.25)
CAM = (3.0, 3.0)
ORTHO, RES_IMG = 6.0, 240
INICIO, ALVO = (0.6, 0.6), (5.4, 5.4)
ANGULOS_SENSORES = np.radians([60, 30, 0, -30, -60])


def pedestre(t):
    return np.array([4.7, 3.4 + 0.9 * math.sin(0.15 * t)]), 0.15


def ocupado_no_mundo(px, py, t, com_oculta=True, com_pedestre=False):
    px, py = np.asarray(px), np.asarray(py)
    hit = np.zeros(px.shape, bool)
    ret = PAREDES + ([CAIXA_OCULTA] if com_oculta else [])
    for (x0, y0, x1, y1) in ret:
        hit |= (px >= x0) & (px <= x1) & (py >= y0) & (py <= y1)
    if com_pedestre:
        c, r = pedestre(t)
        hit |= np.hypot(px - c[0], py - c[1]) <= r
    return hit


def foto_da_camera():
    px = ORTHO / RES_IMG
    cols, rows = np.meshgrid(np.arange(RES_IMG), np.arange(RES_IMG))
    wx = CAM[0] + ORTHO / 2 - (cols + 0.5) * px
    wy = CAM[1] + ORTHO / 2 - (rows + 0.5) * px
    img = np.zeros((RES_IMG, RES_IMG), bool)
    for (x0, y0, x1, y1) in PAREDES:
        img |= (wx >= x0) & (wx <= x1) & (wy >= y0) & (wy <= y1)
    return img


def ler_sensores(x, y, th, t, com_pedestre):
    s = np.linspace(0.02, ROBO.alcance_sensor, 60)
    pts = []
    for a in ANGULOS_SENSORES:
        ox, oy = x + 0.08 * math.cos(th + a), y + 0.08 * math.sin(th + a)
        rx, ry = ox + s * math.cos(th + a), oy + s * math.sin(th + a)
        h = np.nonzero(ocupado_no_mundo(rx, ry, t, True, com_pedestre))[0]
        if len(h):
            pts.append((rx[h[0]], ry[h[0]]))
    return np.array(pts).reshape(-1, 2)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pedestre", action="store_true")
    ap.add_argument("--mostrar", action="store_true")
    ap.add_argument("--saida", default="teste_offline.png")
    args = ap.parse_args()
    vis.preparar(args.mostrar)

    mapa, _ = mapa_de_imagem(foto_da_camera(), CAM[0], CAM[1], ORTHO, MAPA.resolucao_min,
                             ROBO.raio_inflar, MAPA.cam_inverter_x, MAPA.cam_inverter_y,
                             limpar=[(*INICIO, MAPA.raio_limpeza_inicio)])
    nav = NavegadorHibrido(mapa, ROBO, ASTAR, DWA, NAV)
    if not nav.planejar(INICIO, ALVO):
        print("[ERRO] A* nao achou caminho.")
        return 2
    print(f"A*: {nav.astar.expandidos} nos expandidos | caminho de {len(nav.caminho_bruto)} "
          f"celulas -> {len(nav.chaves)} pontos-chave")

    dt = 0.05
    x, y, th = INICIO[0], INICIO[1], math.pi / 4 + math.radians(120)
    v = w = 0.0
    t, dist, colisoes, folga_min = 0.0, 0.0, 0, 9.0
    traj = []
    chegou = False
    while t < NAV.tempo_max:
        obs = ler_sensores(x, y, th, t, args.pedestre)
        v, w, st = nav.passo((x, y, th), v, w, dt, t, obs)
        if st == "CHEGOU":
            chegou = True
            break
        px0, py0 = x, y
        x += v * math.cos(th) * dt
        y += v * math.sin(th) * dt
        th += w * dt
        dist += math.hypot(x - px0, y - py0)
        t += dt
        traj.append((x, y))
        ang = np.linspace(0, 2 * math.pi, 16, endpoint=False)
        if ocupado_no_mundo(x + 0.10 * np.cos(ang), y + 0.10 * np.sin(ang), t, True,
                            args.pedestre).any():
            colisoes += 1
    traj = np.array(traj)

    for (tt, msg) in nav.log:
        print(f"  [t={tt:6.1f}s] {msg}")
    print("-" * 60)
    print(f"Chegou ao alvo : {'SIM' if chegou else 'NAO'}")
    print(f"Tempo simulado : {t:.1f} s | distancia {dist:.2f} m | v media {dist / max(t, 1e-9):.3f} m/s")
    print(f"Replanejamentos: {nav.replans} | passos com contato: {colisoes}")

    vis.mapa_e_rota(nav, INICIO, ALVO, args.saida, trajetoria=traj,
                    retangulos=[CAIXA_OCULTA], titulo="Teste offline: A* + DWA (caixa tracejada = oculta)",
                    mostrar=args.mostrar)
    print(f"Grafico salvo em {args.saida}")
    return 0 if (chegou and colisoes == 0) else 1


if __name__ == "__main__":
    sys.exit(main())
