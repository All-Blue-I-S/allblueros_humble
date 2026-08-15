#!/usr/bin/env python3

import numpy as np

class DVZ:
    """
    Controlador reativo baseado no método das Zonas Virtuais Deformáveis (DVZ) em 3D.

    Esta classe encapsula a modelagem geométrica do elipsoide de segurança,
    o processamento discreto de nuvens de pontos não estruturadas e a extração
    das grandezas de intrusão e jacobianas espaciais.
    """

    def __init__(self, c_min, lambda_c, r0, k_gain, res_deg=5.0):
        self.c_min = np.array(c_min, dtype=float)
        self.lambda_c = np.array(lambda_c, dtype=float)
        self.r0 = np.array(r0, dtype=float)
        self.k_gain = k_gain

        self.d_theta = np.radians(res_deg)
        self.d_phi = np.radians(res_deg)

        # Variáveis da Nuvem Estática (Pré-processadas)
        self._full_u_hat = np.empty((0, 3))
        self._full_d = np.empty(0)
        self._full_w_i = np.empty(0)

        # Estado interno da nuvem de intrusão (Dinâmico)
        self._u_hat_z = np.empty((0, 3))
        self._d_z = np.empty(0)
        self._d_h_z = np.empty(0)
        self._w_i_z = np.empty(0)

        # Variáveis em cache para evitar recálculo
        self.A = np.eye(3)
        self._r0A = np.zeros(3)
        self._c_term = 0.0

        self.update_speed([0.0, 0.0, 0.0])

    def update_speed(self, v):
        """Atualiza a matriz geométrica A e faz pré-cálculos para ganho de performance."""
        v_array = np.array(v, dtype=float)
        c = self.lambda_c * (v_array**2) + self.c_min
        self.A = np.diag(1.0 / (c**2))

        self._r0A = self.r0.dot(self.A)
        self._c_term = np.dot(self._r0A, self.r0) - 1.0

        self._evaluate_intrusion()

    def _compute_weights(self, theta, phi):
        """Calcula pesos espaciais em O(N) usando bincount em vez de unique."""
        idx_theta = (theta / self.d_theta).astype(int)
        idx_phi = (phi / self.d_phi).astype(int)

        max_phi_idx = int(2 * np.pi / self.d_phi) + 1
        voxel_hash = idx_theta * max_phi_idx + idx_phi

        counts = np.bincount(voxel_hash)
        point_counts = counts[voxel_hash]

        d_omega = np.sin(theta) * self.d_theta * self.d_phi
        w_i = d_omega / point_counts

        return w_i

    def _compute_theoretical_distance(self, u_hat):
        """Calcula d_h resolvendo a interseção vetorizada de forma otimizada."""
        Au = u_hat.dot(self.A)

        # OTIMIZAÇÃO: np.einsum é mais rápido que multiplicar matrizes e somar eixos
        a = np.einsum('ij,ij->i', u_hat, Au)

        b = -2.0 * np.dot(u_hat, self._r0A)

        delta = np.maximum(b**2 - 4 * a * self._c_term, 0.0)
        return (-b + np.sqrt(delta)) / (2.0 * a)

    def update_cloud(self, points):
        """Processa a parte geométrica estática da nuvem de pontos."""
        valid_finite = np.isfinite(points).all(axis=1)
        clean_points = points[valid_finite]

        if len(clean_points) == 0:
            self._full_d = np.empty(0)
            return

        norms = np.linalg.norm(clean_points, axis=1)
        valid_mask = norms > 0.001

        pts_valid = clean_points[valid_mask]
        self._full_d = norms[valid_mask]

        if len(pts_valid) == 0:
            self._full_d = np.empty(0)
            return

        x, y, z = pts_valid[:, 0], pts_valid[:, 1], pts_valid[:, 2]

        theta = np.arccos(np.clip(z / self._full_d, -1.0, 1.0))
        phi = np.mod(np.arctan2(y, x), 2 * np.pi)

        self._full_u_hat = pts_valid / self._full_d[:, np.newaxis]
        self._full_w_i = self._compute_weights(theta, phi)

    def _evaluate_intrusion(self):
        """Aplica o filtro d < dh."""
        if len(self._full_d) == 0:
            self._d_z = np.empty(0)
            return

        d_h_full = self._compute_theoretical_distance(self._full_u_hat)

        intrusion_mask = self._full_d < d_h_full

        self._u_hat_z = self._full_u_hat[intrusion_mask]
        self._d_z = self._full_d[intrusion_mask]
        self._d_h_z = d_h_full[intrusion_mask]
        self._w_i_z = self._full_w_i[intrusion_mask]

    def get_intrusion(self):
        """Calcula a Intrusão Total (I)."""
        if len(self._d_z) == 0:
            return 0.0

        intrusion_terms = (self._d_h_z / self._d_z) - 1.0
        return float(np.sum(intrusion_terms * self._w_i_z))

    def get_translational_jacobian(self):
        """Calcula a transposta da Jacobiana Translacional usando BLAS."""
        if len(self._d_z) == 0:
            return np.zeros(3)

        weights = (self._d_h_z / (self._d_z**2)) * self._w_i_z
        return weights @ self._u_hat_z

    def get_rotational_jacobian(self):
        """Calcula a transposta da Jacobiana Rotacional usando BLAS."""
        if len(self._d_z) == 0:
            return np.zeros(3)

        A_u_z = self._u_hat_z.dot(self.A)
        cross_prod = np.cross(self._u_hat_z, A_u_z)

        weights = ((self._d_h_z**3) / self._d_z) * self._w_i_z
        return weights @ cross_prod

    def get_control_velocities(self):
        """Sintetiza as leis de controle cinemático finais."""
        I_total = self.get_intrusion()
        if I_total == 0.0:
            return np.zeros(3), np.zeros(3)

        J_v_T = self.get_translational_jacobian()
        J_omega_T = self.get_rotational_jacobian()

        v_zvd = -self.k_gain * I_total * J_v_T
        omega_zvd = -self.k_gain * I_total * J_omega_T

        return np.nan_to_num(v_zvd), np.nan_to_num(omega_zvd)
