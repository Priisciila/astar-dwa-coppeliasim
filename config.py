import math
from dataclasses import dataclass

CENA = {
    "robo": "/Cuboid",
    "motor_esq": "/Cuboid/MOTOR_ESQUERDO",
    "motor_dir": "/Cuboid/MOTOR_DIREITO",
    "sensores": [
        "/Cuboid/SENSOR_ESQUERDO",
        "/Cuboid/SENSOR_DIAG_ESQUERDO",
        "/Cuboid/SENSOR_MEIO",
        "/Cuboid/SENSOR_DIAG_DIREITO",
        "/Cuboid/SENSOR_DIREITO",
    ],
    "alvo": "/Target",
    "camera": "/Vision_sensor",
}


@dataclass
class ParametrosRobo:
    raio_roda: float = 0.036
    dist_rodas: float = 0.235
    offset_theta: float = math.pi
    sinal_w: float = 1.0
    raio_colisao: float = 0.12
    raio_inflar: float = 0.15
    v_max: float = 0.15
    v_min: float = 0.0
    w_max: float = 0.8
    a_v: float = 0.3
    a_w: float = 1.2
    alcance_sensor: float = 0.5


@dataclass
class ParametrosMapa:
    limiar_binario: int = 245
    resolucao_min: float = 0.15
    raio_limpeza_inicio: float = 0.35
    raio_limpeza_alvo: float = 0.20
    cam_inverter_x: bool = True
    cam_inverter_y: bool = True
    plot_inverter_eixos: bool = True


@dataclass
class ParametrosAStar:
    vizinhanca: int = 16
    peso_min: float = 0.5
    peso_max: float = 1.5
    raio_densidade: int = 3
    passo_denso: float = 0.05


@dataclass
class ParametrosDWA:
    tempo_predicao: float = 1.2
    dt_predicao: float = 0.1
    n_v: int = 7
    n_w: int = 21
    alpha: float = 2.0
    beta: float = 1.2
    gamma: float = 1.0
    delta: float = 1.5
    epsilon: float = 0.1
    folga_ref: float = 0.35
    dist_ref_caminho: float = 0.5
    a_freio: float = 0.5


@dataclass
class ParametrosNav:
    tol_subobj: float = 0.25
    tol_final: float = 0.20
    janela_travado: float = 8.0
    deslocamento_travado: float = 0.08
    desvio_replan: float = 0.6
    intervalo_replan: float = 3.0
    max_replans: int = 5
    memoria_obstaculos: float = 6.0
    dur_recuperacao: float = 1.5
    w_recuperacao: float = 0.6
    v_recuperacao: float = -0.03
    tempo_max: float = 900.0


ROBO = ParametrosRobo()
MAPA = ParametrosMapa()
ASTAR = ParametrosAStar()
DWA = ParametrosDWA()
NAV = ParametrosNav()
