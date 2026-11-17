import math
from collections import deque

import numpy as np

from dwa_melhorado import DWAMelhorado
from planejador_astar import AStarMelhorado, densificar


class NavegadorHibrido:
    def __init__(self, mapa, robo, p_astar, p_dwa, p_nav):
        self.mapa, self.robo = mapa, robo
        self.pa, self.pn = p_astar, p_nav
        self.astar = AStarMelhorado(mapa, p_astar)
        self.dwa = DWAMelhorado(robo, p_dwa)

        self.alvo = None
        self.chaves, self.denso, self.idx_chave = [], np.zeros((0, 2)), []
        self.caminho_bruto = np.zeros((0, 2))
        self.i_sub, self.i_proj = 1, 0

        self.memoria = {}
        self.hist = deque()
        self.replans, self.travados = 0, 0
        self._t_replan = -1e9
        self._rec_ate, self._rec_v, self._rec_w = -1.0, 0.0, 0.0
        self.log = []

    def planejar(self, inicio, alvo):
        res = self.astar.planejar(inicio, alvo)
        if res is None:
            return False
        mundo, idx = res
        m = self.mapa
        chaves = [m.para_mundo(*c) for c in self.astar.extrair_pontos_chave(idx)]
        chaves[0] = (float(inicio[0]), float(inicio[1]))
        if len(chaves) == 1:
            chaves.append((float(alvo[0]), float(alvo[1])))
        elif m.livre(*m.para_indice(*alvo)):
            chaves[-1] = (float(alvo[0]), float(alvo[1]))
        self.alvo = (float(alvo[0]), float(alvo[1]))
        self.caminho_bruto = np.array(mundo)
        self.chaves = chaves
        self.denso, self.idx_chave = densificar(chaves, self.pa.passo_denso)
        self.i_sub, self.i_proj = 1, 0
        return True

    def _replanejar(self, pos, t, motivo):
        validas = [c for c, tt in self.memoria.items() if t - tt < self.pn.memoria_obstaculos]
        self.mapa.definir_dinamicos(validas)
        ok = self.planejar((pos[0], pos[1]), self.alvo)
        if not ok:
            self.mapa.definir_dinamicos([])
            ok = self.planejar((pos[0], pos[1]), self.alvo)
        self.replans += 1
        self._t_replan = t
        self.hist.clear()
        self.log.append((t, f"REPLANEJAMENTO #{self.replans} ({motivo}) -> "
                            f"{'ok' if ok else 'falhou'}, {len(self.chaves)} pontos-chave"))
        return ok

    def _registrar_desconhecidos(self, obstaculos, t):
        for px, py in np.asarray(obstaculos, float).reshape(-1, 2):
            c = self.mapa.para_indice(px, py)
            if self.mapa.dentro(*c) and not self.mapa.conhecido(px, py):
                self.memoria[c] = t

    def _atualizar_projecao(self, pos):
        i0, i1 = self.i_proj, min(self.i_proj + 60, len(self.denso))
        d = np.hypot(self.denso[i0:i1, 0] - pos[0], self.denso[i0:i1, 1] - pos[1])
        self.i_proj = i0 + int(np.argmin(d))

    def _lados_livres(self, pose, obstaculos):
        x, y, th = pose
        esq = dirr = self.robo.alcance_sensor
        frente = self.robo.alcance_sensor
        for ox, oy in np.asarray(obstaculos, float).reshape(-1, 2):
            dx, dy = ox - x, oy - y
            fx = math.cos(th) * dx + math.sin(th) * dy
            fy = -math.sin(th) * dx + math.cos(th) * dy
            d = math.hypot(dx, dy)
            if fy > 0:
                esq = min(esq, d)
            else:
                dirr = min(dirr, d)
            if fx > 0 and abs(fy) < 0.15:
                frente = min(frente, d)
        return esq, dirr, frente

    def _iniciar_recuperacao(self, t, pose, obstaculos, motivo):
        esq, dirr, frente = self._lados_livres(pose, obstaculos)
        sinal = 1.0 if esq >= dirr else -1.0
        self._rec_w = sinal * self.pn.w_recuperacao
        self._rec_v = self.pn.v_recuperacao if frente < 0.20 else 0.0
        self._rec_ate = t + self.pn.dur_recuperacao
        self.hist.clear()
        self.log.append((t, f"RECUPERACAO ({motivo}): w={self._rec_w:+.2f} v={self._rec_v:+.2f}"))

    def passo(self, pose, v, w, dt, t, obstaculos):
        x, y, th = pose
        pos = np.array([x, y])
        meta = self.chaves[-1]
        if math.hypot(meta[0] - x, meta[1] - y) < self.pn.tol_final:
            return 0.0, 0.0, "CHEGOU"

        self._registrar_desconhecidos(obstaculos, t)
        self._atualizar_projecao(pos)

        while self.i_sub < len(self.chaves) - 1:
            k = self.chaves[self.i_sub]
            if (math.hypot(k[0] - x, k[1] - y) < self.pn.tol_subobj
                    or self.i_proj >= self.idx_chave[self.i_sub]):
                self.log.append((t, f"sub-objetivo {self.i_sub}/{len(self.chaves) - 1} cumprido"))
                self.i_sub += 1
            else:
                break
        sub = self.chaves[self.i_sub]

        if t < self._rec_ate:
            return self._rec_v, self._rec_w, "RECUPERACAO"

        self.hist.append((t, x, y))
        if t - self.hist[0][0] >= self.pn.janela_travado:
            _, x0, y0 = self.hist.popleft()
            if math.hypot(x - x0, y - y0) < self.pn.deslocamento_travado:
                self.travados += 1
                if self.travados % 2 == 0 and self.replans < self.pn.max_replans:
                    self._replanejar(pos, t, "travado")
                else:
                    self._iniciar_recuperacao(t, pose, obstaculos, "travado")
                    return self._rec_v, self._rec_w, "RECUPERACAO"

        desvio = float(np.hypot(*(self.denso[self.i_proj] - pos)))
        recentes = any(t - tt < self.pn.memoria_obstaculos for tt in self.memoria.values())
        if (desvio > self.pn.desvio_replan and recentes
                and t - self._t_replan > self.pn.intervalo_replan
                and self.replans < self.pn.max_replans):
            self._replanejar(pos, t, f"desvio {desvio:.2f} m")
            sub = self.chaves[self.i_sub]

        ref = self.denso[self.i_proj:self.i_proj + 80]
        res = self.dwa.calcular(pose, v, w, dt, sub, obstaculos, ref)
        if res is None:
            self._iniciar_recuperacao(t, pose, obstaculos, "sem comando seguro")
            return self._rec_v, self._rec_w, "RECUPERACAO"
        return res[0], res[1], "DWA"
