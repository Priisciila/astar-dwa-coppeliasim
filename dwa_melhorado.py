import math

import numpy as np


def normalizar_angulo(a):
    return math.atan2(math.sin(a), math.cos(a))


class DWAMelhorado:
    def __init__(self, robo, params):
        self.robo = robo
        self.p = params

    def calcular(self, pose, v, w, dt, subobj, obstaculos, caminho_ref=None):
        r, p = self.robo, self.p
        x0, y0, th0 = pose
        v = min(max(v, r.v_min), r.v_max)
        w = min(max(w, -r.w_max), r.w_max)

        v_lo, v_hi = max(r.v_min, v - r.a_v * dt), min(r.v_max, v + r.a_v * dt)
        w_lo, w_hi = max(-r.w_max, w - r.a_w * dt), min(r.w_max, w + r.a_w * dt)
        vs = np.linspace(v_lo, v_hi, p.n_v)
        ws = np.linspace(w_lo, w_hi, p.n_w)
        if w_lo < 0.0 < w_hi:
            ws = np.sort(np.append(ws, 0.0))
        VV, WW = np.meshgrid(vs, ws, indexing="ij")
        VV, WW = VV.ravel(), WW.ravel()

        K = max(int(round(p.tempo_predicao / p.dt_predicao)), 1)
        t = np.arange(1, K + 1) * p.dt_predicao
        V, W = VV[:, None], WW[:, None]
        peq = np.abs(W) < 1e-4
        Ws = np.where(peq, 1.0, W)
        th = th0 + W * t
        X = np.where(peq, x0 + V * t * math.cos(th0),
                     x0 + V / Ws * (np.sin(th) - math.sin(th0)))
        Y = np.where(peq, y0 + V * t * math.sin(th0),
                     y0 - V / Ws * (np.cos(th) - math.cos(th0)))

        obs = np.asarray(obstaculos, dtype=float).reshape(-1, 2)
        if len(obs):
            d = np.hypot(X[:, :, None] - obs[None, None, :, 0],
                         Y[:, :, None] - obs[None, None, :, 1])
            folga = d.min(axis=(1, 2)) - r.raio_colisao
            d_atual = float(np.hypot(x0 - obs[:, 0], y0 - obs[:, 1]).min())
        else:
            folga = np.full(len(VV), np.inf)
            d_atual = r.alcance_sensor
        folga_atual = d_atual - r.raio_colisao
        ok = folga >= min(0.0, folga_atual) - 1e-9
        ok &= VV <= np.sqrt(2.0 * np.maximum(folga, 0.0) * p.a_freio) + 1e-9

        ang = np.arctan2(subobj[1] - Y[:, -1], subobj[0] - X[:, -1])
        err = np.abs(np.arctan2(np.sin(ang - th[:, -1]), np.cos(ang - th[:, -1])))
        s_head = 1.0 - err / math.pi

        if caminho_ref is not None and len(caminho_ref) and p.delta > 0:
            ref = np.asarray(caminho_ref, dtype=float)
            dp = np.hypot(X[:, -1, None] - ref[None, :, 0],
                          Y[:, -1, None] - ref[None, :, 1]).min(axis=1)
            s_path = 1.0 - np.clip(dp / p.dist_ref_caminho, 0.0, 1.0)
        else:
            s_path = np.zeros(len(VV))

        s_clear = np.clip(folga / p.folga_ref, 0.0, 1.0)
        err_atual = abs(normalizar_angulo(math.atan2(subobj[1] - y0, subobj[0] - x0) - th0))
        alinhado = max(0.0, math.cos(err_atual))
        s_vel = VV / r.v_max * alinhado
        s_suave = 1.0 - np.abs(WW - w) / (2.0 * r.w_max)

        f = min(max((d_atual - 0.15) / max(r.alcance_sensor - 0.15, 1e-6), 0.0), 1.0)
        beta = p.beta * (2.0 - f)
        gamma = p.gamma * (0.5 + 0.5 * f)

        score = (p.alpha * s_head + beta * s_clear + gamma * s_vel
                 + p.delta * s_path + p.epsilon * s_suave)
        score = np.where(ok, score, -np.inf)
        if not np.isfinite(score).any():
            return None
        b = int(np.argmax(score))
        info = {"score": float(score[b]), "folga": float(folga[b]), "d_obs": d_atual,
                "traj": (X[b].copy(), Y[b].copy())}
        return float(VV[b]), float(WW[b]), info
