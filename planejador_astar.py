import heapq
import math

import numpy as np

VIZINHANCA_8 = [(1, 0), (0, 1), (-1, 0), (0, -1), (1, 1), (-1, 1), (1, -1), (-1, -1)]
VIZINHANCA_16 = VIZINHANCA_8 + [
    (2, 1), (1, 2), (-1, 2), (-2, 1), (-2, -1), (-1, -2), (1, -2), (2, -1),
]


def _deslocar(a, dx, dy):
    nx, ny = a.shape
    out = np.zeros_like(a)
    sx = slice(max(0, -dx), nx - max(0, dx))
    dxs = slice(max(0, dx), nx - max(0, -dx))
    sy = slice(max(0, -dy), ny - max(0, dy))
    dys = slice(max(0, dy), ny - max(0, -dy))
    out[dxs, dys] = a[sx, sy]
    return out


class MapaOcupacao:

    def __init__(self, bruto, resolucao, x_min, y_min, raio_inflar, raio_densidade=3):
        self.bruto = np.asarray(bruto, dtype=bool)
        self.dinamico = np.zeros_like(self.bruto)
        self.res = float(resolucao)
        self.x_min = float(x_min)
        self.y_min = float(y_min)
        self.raio_inflar = float(raio_inflar)
        self.raio_densidade = int(raio_densidade)
        self.nx, self.ny = self.bruto.shape
        self._recalcular()

    def _recalcular(self):
        base = self.bruto | self.dinamico
        r = int(math.ceil(self.raio_inflar / self.res))
        inflado = base.copy()
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                if (dx or dy) and math.hypot(dx, dy) * self.res <= self.raio_inflar + 1e-9:
                    inflado |= _deslocar(base, dx, dy)
        self.ocupado = inflado
        self.densidade = self._densidade_local()

    def _densidade_local(self):
        r = self.raio_densidade
        k = 2 * r + 1
        a = np.pad(self.ocupado.astype(np.float64), r)
        c = np.pad(a.cumsum(0).cumsum(1), ((1, 0), (1, 0)))
        soma = c[k:, k:] - c[:-k, k:] - c[k:, :-k] + c[:-k, :-k]
        return soma / (k * k)

    def definir_dinamicos(self, celulas):
        self.dinamico[:] = False
        for ix, iy in celulas:
            if self.dentro(ix, iy):
                self.dinamico[ix, iy] = True
        self._recalcular()

    def para_indice(self, x, y):
        return (int(round((x - self.x_min) / self.res)),
                int(round((y - self.y_min) / self.res)))

    def para_mundo(self, ix, iy):
        return (self.x_min + ix * self.res, self.y_min + iy * self.res)

    def dentro(self, ix, iy):
        return 0 <= ix < self.nx and 0 <= iy < self.ny

    def livre(self, ix, iy):
        return self.dentro(ix, iy) and not self.ocupado[ix, iy]

    def conhecido(self, x, y):
        ix, iy = self.para_indice(x, y)
        x0, x1 = max(ix - 1, 0), min(ix + 2, self.nx)
        y0, y1 = max(iy - 1, 0), min(iy + 2, self.ny)
        return bool(self.bruto[x0:x1, y0:y1].any())

    def segmento_livre(self, a, b):
        dist = math.hypot(b[0] - a[0], b[1] - a[1])
        n = max(int(math.ceil(dist / 0.5)), 1)
        for k in range(n + 1):
            t = k / n
            ix = int(round(a[0] + t * (b[0] - a[0])))
            iy = int(round(a[1] + t * (b[1] - a[1])))
            if not self.livre(ix, iy):
                return False
        return True

    def celula_livre_mais_proxima(self, c, raio_max=8):
        if self.livre(*c):
            return c
        melhor, dmelhor = None, float("inf")
        for dx in range(-raio_max, raio_max + 1):
            for dy in range(-raio_max, raio_max + 1):
                nc = (c[0] + dx, c[1] + dy)
                d = math.hypot(dx, dy)
                if d < dmelhor and self.livre(*nc):
                    melhor, dmelhor = nc, d
        return melhor


def mapa_de_imagem(binario, cam_x, cam_y, ortho, resolucao_min, raio_inflar,
                   inverter_x=True, inverter_y=True, limpar=(), raio_densidade=3):
    res_y, res_x = binario.shape
    px = ortho / max(res_x, res_y)
    meio_x, meio_y = res_x * px / 2.0, res_y * px / 2.0
    linhas, colunas = np.nonzero(binario)
    cx, cy = (colunas + 0.5) * px, (linhas + 0.5) * px
    wx = cam_x + meio_x - cx if inverter_x else cam_x - meio_x + cx
    wy = cam_y + meio_y - cy if inverter_y else cam_y - meio_y + cy

    manter = np.ones(len(wx), dtype=bool)
    for (lx, ly, lr) in limpar:
        manter &= np.hypot(wx - lx, wy - ly) > lr
    wx, wy = wx[manter], wy[manter]

    res = max(2 * px, resolucao_min)
    x_min, y_min = cam_x - meio_x, cam_y - meio_y
    nx = int(math.ceil(2 * meio_x / res)) + 1
    ny = int(math.ceil(2 * meio_y / res)) + 1
    bruto = np.zeros((nx, ny), dtype=bool)
    ix = np.clip(np.rint((wx - x_min) / res).astype(int), 0, nx - 1)
    iy = np.clip(np.rint((wy - y_min) / res).astype(int), 0, ny - 1)
    bruto[ix, iy] = True
    mapa = MapaOcupacao(bruto, res, x_min, y_min, raio_inflar, raio_densidade)
    return mapa, (wx, wy)


class AStarMelhorado:
    def __init__(self, mapa, params):
        self.mapa = mapa
        self.p = params
        self.mov = VIZINHANCA_16 if params.vizinhanca == 16 else VIZINHANCA_8
        self.expandidos = 0

    def _h(self, c, g):
        dens = self.mapa.densidade[c[0], c[1]]
        peso = min(self.p.peso_max, max(self.p.peso_min, self.p.peso_max - dens))
        return peso * math.hypot(c[0] - g[0], c[1] - g[1])

    def planejar(self, inicio, alvo):
        m = self.mapa
        s = m.celula_livre_mais_proxima(m.para_indice(*inicio))
        g = m.celula_livre_mais_proxima(m.para_indice(*alvo))
        if s is None or g is None:
            return None

        custo = {s: 0.0}
        pai = {}
        heap = [(self._h(s, g), 0.0, s)]
        fechado = set()
        achou = False
        self.expandidos = 0
        while heap:
            _, gc, cur = heapq.heappop(heap)
            if cur in fechado:
                continue
            fechado.add(cur)
            self.expandidos += 1
            if cur == g:
                achou = True
                break
            for dx, dy in self.mov:
                nb = (cur[0] + dx, cur[1] + dy)
                if nb in fechado or not m.livre(*nb) or not m.segmento_livre(cur, nb):
                    continue
                ng = gc + math.hypot(dx, dy)
                if ng < custo.get(nb, float("inf")):
                    custo[nb] = ng
                    pai[nb] = cur
                    heapq.heappush(heap, (ng + self._h(nb, g), ng, nb))
        if not achou:
            return None

        cam = [g]
        while cam[-1] != s:
            cam.append(pai[cam[-1]])
        cam.reverse()
        return [m.para_mundo(*c) for c in cam], cam

    def extrair_pontos_chave(self, cam):
        m = self.mapa
        if len(cam) <= 2:
            return list(cam)
        vertices = [cam[0]]
        for i in range(1, len(cam) - 1):
            d1 = (cam[i][0] - cam[i - 1][0], cam[i][1] - cam[i - 1][1])
            d2 = (cam[i + 1][0] - cam[i][0], cam[i + 1][1] - cam[i][1])
            cruz = d1[0] * d2[1] - d1[1] * d2[0]
            ponto = d1[0] * d2[0] + d1[1] * d2[1]
            if cruz != 0 or ponto < 0:
                vertices.append(cam[i])
        vertices.append(cam[-1])

        chaves = [vertices[0]]
        i = 0
        while i < len(vertices) - 1:
            j = len(vertices) - 1
            while j > i + 1 and not m.segmento_livre(vertices[i], vertices[j]):
                j -= 1
            chaves.append(vertices[j])
            i = j
        return chaves


def densificar(pontos, passo):
    denso, idx = [], []
    for k in range(len(pontos) - 1):
        a, b = np.array(pontos[k], float), np.array(pontos[k + 1], float)
        n = max(int(math.ceil(np.linalg.norm(b - a) / passo)), 1)
        idx.append(len(denso))
        for i in range(n):
            denso.append(a + (b - a) * i / n)
    denso.append(np.array(pontos[-1], float))
    idx.append(len(denso) - 1)
    return np.array(denso), idx
