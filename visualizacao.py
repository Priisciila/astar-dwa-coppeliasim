import matplotlib

plt = None


def preparar(mostrar=False):
    global plt
    if not mostrar:
        matplotlib.use("Agg")
    import matplotlib.pyplot as _plt
    plt = _plt


def _finalizar(fig, arquivo, mostrar):
    fig.tight_layout()
    fig.savefig(arquivo, dpi=130)
    if mostrar:
        plt.show()
    plt.close(fig)


def imagens_camera(cinza, binario, arquivo, mostrar=False):
    fig, ax = plt.subplots(1, 2, figsize=(10, 5))
    ax[0].imshow(cinza, cmap="gray", origin="lower")
    ax[0].set_title("Visao bruta da camera")
    ax[1].imshow(binario, cmap="gray", origin="lower")
    ax[1].set_title("Mapa binario (branco = obstaculo)")
    _finalizar(fig, arquivo, mostrar)


def mapa_e_rota(nav, inicio, alvo, arquivo, trajetoria=None, retangulos=(),
                inverter_eixos=False, titulo="A* melhorado + DWA", mostrar=False):
    m = nav.mapa
    fig, ax = plt.subplots(figsize=(9, 8))
    ext = [m.x_min - m.res / 2, m.x_min + (m.nx - 0.5) * m.res,
           m.y_min - m.res / 2, m.y_min + (m.ny - 0.5) * m.res]
    ax.imshow(m.ocupado.T, origin="lower", extent=ext, cmap="Greys", alpha=0.35)
    bx, by = m.bruto.nonzero()
    ax.plot(m.x_min + bx * m.res, m.y_min + by * m.res, "s", color="k", ms=3, label="obstaculos")
    for (x0, y0, x1, y1) in retangulos:
        ax.add_patch(plt.Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, ls="--", ec="r"))
    if len(nav.caminho_bruto):
        ax.plot(nav.caminho_bruto[:, 0], nav.caminho_bruto[:, 1], "--", c="lightcoral",
                lw=1.5, label="A* (grade)")
    ch = nav.chaves
    ax.plot([c[0] for c in ch], [c[1] for c in ch], "-", c="magenta", lw=3, label="pontos-chave")
    ax.scatter([c[0] for c in ch[1:-1]], [c[1] for c in ch[1:-1]], c="saddlebrown", s=90, zorder=5)
    if trajetoria is not None and len(trajetoria):
        ax.plot(trajetoria[:, 0], trajetoria[:, 1], "-", c="green", lw=2.5, label="trajetoria real")
    ax.plot(*inicio[:2], "go", ms=13, label="inicio")
    ax.plot(*alvo[:2], "bX", ms=14, label="alvo")
    ax.set_aspect("equal")
    ax.grid(True, ls=":", alpha=0.4)
    ax.legend(loc="best", fontsize=8)
    ax.set_title(titulo)
    if inverter_eixos:
        ax.invert_xaxis()
        ax.invert_yaxis()
    _finalizar(fig, arquivo, mostrar)
