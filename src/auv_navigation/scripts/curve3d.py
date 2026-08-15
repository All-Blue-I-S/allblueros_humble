#!/usr/bin/env python3

import numpy as np
from scipy.interpolate import CubicSpline, PchipInterpolator, interp1d
from scipy.spatial.transform import Rotation

class Curve3D:
    """
    Representação contínua diferenciável de uma curva 3D parametrizada por
    um parâmetro arbitrário lambda (lam).

    A parametrização interna utiliza o método Centrípeta por padrão para
    suavização de splines.

    Fornece:
        - Propriedades Analíticas: p(lam), p'(lam), p''(lam), p'''(lam)
        - Geometria Pura (Frenet): Tangente, Normal, Binormal, Curvatura
        - Referenciais de Controle: Quaternion de Voo Nivelado (Z-Up)
        - Cinemática Exata 6-DOF
    """

    def __init__(self, waypoints: np.ndarray, method: str = "cubic"):
        """
        Parameters
        ----------
        waypoints : np.ndarray (N x 3)
            Pontos 3D da curva.
        method : str
            'linear', 'pchip' ou 'cubic'
        """
        if waypoints.shape[0] < 2:
            raise ValueError("São necessários pelo menos dois waypoints.")

        self.method = method.lower()

        # Remove duplicados consecutivos
        mask = np.ones(len(waypoints), dtype=bool)
        mask[1:] = np.any(np.diff(waypoints, axis=0) != 0.0, axis=1)
        self.waypoints = waypoints[mask]

        if len(self.waypoints) < 2:
            raise ValueError("Waypoints inválidos após remoção de duplicados.")

        # Parametrização Centrípeta (Robusta contra overshoot de spline)
        diffs = np.diff(self.waypoints, axis=0)
        dists = np.linalg.norm(diffs, axis=1)
        self.lam_nodes = np.concatenate(([0.0], np.cumsum(np.sqrt(dists))))
        self.length = self.lam_nodes[-1]

        if self.length <= 1e-9:
            raise ValueError("Comprimento total da curva é zero.")

        # Criar interpoladores
        self._build_interpolators()

    # ============================================================
    # Construção dos interpoladores (Até a 3ª Ordem)
    # ============================================================

    def _build_interpolators(self):

        x = self.waypoints[:, 0]
        y = self.waypoints[:, 1]
        z = self.waypoints[:, 2]

        if self.method == "linear":
            self.fx = interp1d(self.lam_nodes, x, kind="linear", fill_value="extrapolate")
            self.fy = interp1d(self.lam_nodes, y, kind="linear", fill_value="extrapolate")
            self.fz = interp1d(self.lam_nodes, z, kind="linear", fill_value="extrapolate")

            # derivadas aproximadas
            self.fx_d1 = lambda lam: np.gradient(self.fx(lam), lam)
            self.fy_d1 = lambda lam: np.gradient(self.fy(lam), lam)
            self.fz_d1 = lambda lam: np.gradient(self.fz(lam), lam)

            self.fx_d2 = lambda lam: np.zeros_like(lam)
            self.fy_d2 = lambda lam: np.zeros_like(lam)
            self.fz_d2 = lambda lam: np.zeros_like(lam)

            self.fx_d3 = lambda lam: np.zeros_like(lam)
            self.fy_d3 = lambda lam: np.zeros_like(lam)
            self.fz_d3 = lambda lam: np.zeros_like(lam)

        elif self.method == "pchip":
            self.fx = PchipInterpolator(self.lam_nodes, x)
            self.fy = PchipInterpolator(self.lam_nodes, y)
            self.fz = PchipInterpolator(self.lam_nodes, z)

            self.fx_d1, self.fx_d2, self.fx_d3 = [self.fx.derivative(i) for i in (1, 2, 3)]
            self.fy_d1, self.fy_d2, self.fy_d3 = [self.fy.derivative(i) for i in (1, 2, 3)]
            self.fz_d1, self.fz_d2, self.fz_d3 = [self.fz.derivative(i) for i in (1, 2, 3)]

        else:  # cubic spline C2
            self.fx = CubicSpline(self.lam_nodes, x)
            self.fy = CubicSpline(self.lam_nodes, y)
            self.fz = CubicSpline(self.lam_nodes, z)

            self.fx_d1, self.fx_d2, self.fx_d3 = [self.fx.derivative(i) for i in (1, 2, 3)]
            self.fy_d1, self.fy_d2, self.fy_d3 = [self.fy.derivative(i) for i in (1, 2, 3)]
            self.fz_d1, self.fz_d2, self.fz_d3 = [self.fz.derivative(i) for i in (1, 2, 3)]

    # ============================================================
    # Avaliações Analíticas
    # ============================================================

    def position(self, lam):
        lam = np.atleast_1d(np.clip(lam, 0.0, self.length))
        return np.vstack((self.fx(lam), self.fy(lam), self.fz(lam))).T

    def first_derivative(self, lam):
        lam = np.atleast_1d(np.clip(lam, 0.0, self.length))
        return np.vstack((self.fx_d1(lam), self.fy_d1(lam), self.fz_d1(lam))).T

    def second_derivative(self, lam):
        lam = np.atleast_1d(np.clip(lam, 0.0, self.length))
        return np.vstack((self.fx_d2(lam), self.fy_d2(lam), self.fz_d2(lam))).T

    def third_derivative(self, lam):
        lam = np.atleast_1d(np.clip(lam, 0.0, self.length))
        return np.vstack((self.fx_d3(lam), self.fy_d3(lam), self.fz_d3(lam))).T

    # ============================================================
    # Avaliações Auxiliares
    # ============================================================

    def get_next_waypoint_index(self, lam):
        """
        Mapeia um ou mais parâmetros (lam) de volta para a topologia original da curva.
        Retorna o índice do waypoint global imediatamente à frente do ponto atual.

        Args:
            lam (float ou np.ndarray): Parâmetro(s) da curva a serem avaliados.

        Returns:
            np.ndarray (int): Índices dos próximos waypoints originais.
        """
        lam = np.atleast_1d(np.clip(lam, 0.0, self.length))
        indices = np.searchsorted(self.lam_nodes, lam, side='right')
        max_idx = len(self.lam_nodes) - 1
        indices = np.clip(indices, 1, max_idx)

        return indices

    # ============================================================
    # Geometria Pura (Frenet Frame)
    # ============================================================

    def tangent(self, lam):
        dp = self.first_derivative(lam)
        norm = np.linalg.norm(dp, axis=1, keepdims=True)
        norm[norm < 1e-9] = 1e-9
        return dp / norm

    def curvature(self, lam):
        dp = self.first_derivative(lam)
        ddp = self.second_derivative(lam)

        cross = np.cross(dp, ddp)
        num = np.linalg.norm(cross, axis=1)
        den = np.linalg.norm(dp, axis=1) ** 3

        den[den < 1e-12] = 1e-12
        return num / den

    def normal(self, lam):
        t = self.tangent(lam)
        ddp = self.second_derivative(lam)

        proj = np.sum(ddp * t, axis=1, keepdims=True) * t
        n = ddp - proj

        norm = np.linalg.norm(n, axis=1, keepdims=True)

        mask = norm[:, 0] < 1e-9
        if np.any(mask):
            arbitrary = np.array([1.0, 0.0, 0.0])
            alt = np.cross(t[mask], arbitrary)
            alt_norm = np.linalg.norm(alt, axis=1, keepdims=True)
            alt_norm[alt_norm < 1e-9] = 1e-9
            n[mask] = alt / alt_norm
            norm[mask] = 1.0

        return n / norm

    def binormal(self, lam):
        t = self.tangent(lam)
        n = self.normal(lam)
        b = np.cross(t, n)

        norm = np.linalg.norm(b, axis=1, keepdims=True)
        norm[norm < 1e-9] = 1e-9

        return b / norm

    def frenet_frame(self, lam):
        return self.tangent(lam), self.normal(lam), self.binormal(lam)

    def centripetal_acceleration(self, lam, v):
        kappa = self.curvature(lam).reshape(-1, 1)
        n = self.normal(lam)
        return (v ** 2) * kappa * n

    def angular_velocity(self, lam, v):
        kappa = self.curvature(lam).reshape(-1, 1)
        b = self.binormal(lam)
        return v * kappa * b

    # ============================================================
    # Referenciais de Controle e Cinemática AUV (Voo Nivelado)
    # ============================================================

    def level_flight_quaternion(self, lam):
        """
        Calcula a orientação garantindo que as asas permaneçam
        paralelas ao horizonte (Z-Up).
        Retorna quaternions no formato [w, x, y, z] para controle.
        """
        tang = self.tangent(lam)
        N_pts = tang.shape[0]
        z_in = np.array([0.0, 0.0, 1.0])

        x_b = tang
        v_lat = np.cross(z_in, x_b)
        norm_v_lat = np.linalg.norm(v_lat, axis=1, keepdims=True)

        y_b = np.zeros_like(v_lat)
        valid = (norm_v_lat > 1e-6).flatten()

        # Atribuição normal
        y_b[valid] = v_lat[valid] / norm_v_lat[valid]

        # Tratamento de singularidade (Gimbal Lock)
        last_y = np.array([0.0, 1.0, 0.0])
        for i in range(N_pts):
            if valid[i]:
                last_y = y_b[i]
            else:
                y_b[i] = last_y

        z_b = np.cross(x_b, y_b)

        # Matriz de Rotação (N, 3, 3) e conversão via SciPy
        R_d = np.stack((x_b, y_b, z_b), axis=-1)
        rot = Rotation.from_matrix(R_d)

        return rot.as_quat()[:, [3, 0, 1, 2]]

    def kinematic_references(self, lam, vel_ref: float):
        """
        Extrai o estado cinemático 6-DOF vetorizado, cancelando as
        distorções paramétricas usando a regra da cadeia exata.

        Returns:
            pos, quat_wxyz, vel_lin, vel_ang, acc_lin, acc_ang
        """
        pos = self.position(lam)
        rp = self.first_derivative(lam)
        rddp = self.second_derivative(lam)
        rdddp = self.third_derivative(lam)

        nrp = np.linalg.norm(rp, axis=1, keepdims=True)
        nrp[nrp < 1e-9] = 1e-9

        tang = rp / nrp
        dot_rp_rdp = np.sum(rp * rddp, axis=1, keepdims=True)
        d_nrp_dl = dot_rp_rdp / nrp

        lam_dot = vel_ref / nrp
        lam_ddot = -(vel_ref * dot_rp_rdp / (nrp**3)) * lam_dot

        dtang_dl = (rddp / nrp) - tang * (dot_rp_rdp / (nrp**2))
        tang_dot = dtang_dl * lam_dot

        term_A_prime = (rdddp / nrp) - (rddp * d_nrp_dl / (nrp**2))
        norm_rddp_sq = np.linalg.norm(rddp, axis=1, keepdims=True)**2
        dot_rp_rdddp = np.sum(rp * rdddp, axis=1, keepdims=True)

        term_B_prime = (rddp * dot_rp_rdp / (nrp**3)) + \
                       (rp * (norm_rddp_sq + dot_rp_rdddp) / (nrp**3)) - \
                       (3 * rp * (dot_rp_rdp**2) / (nrp**5))

        d2tang_dl2 = term_A_prime - term_B_prime
        tang_ddot = d2tang_dl2 * (lam_dot**2) + dtang_dl * lam_ddot

        vel_lin = vel_ref * tang
        acc_lin = vel_ref * tang_dot
        vel_ang = np.cross(tang, tang_dot)
        acc_ang = np.cross(tang, tang_ddot)

        quat_wxyz = self.level_flight_quaternion(lam)

        return pos, quat_wxyz, vel_lin, vel_ang, acc_lin, acc_ang
