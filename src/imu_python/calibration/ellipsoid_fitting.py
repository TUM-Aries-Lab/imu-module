"""Ellipsoid fitting algorithms."""

from dataclasses import dataclass
from enum import StrEnum

import numpy as np
from numpy.typing import NDArray
from scipy import linalg

from imu_python.definitions import MAGNETIC_FIELD_STRENGTH


class FittingAlgorithmNames(StrEnum):
    """Enum for ellipsoid fitting algorithms."""

    LS = "LS"
    MLE = "MLE"


@dataclass
class FittingAlgorithm:
    """Base class for ellipsoid fitting algorithms.

    Attributes
    ----------
    hard_iron: np.ndarray (3,)
        Estimated hard-iron offset vector
    inv_soft_iron: np.ndarray (3,3)
        Estimated inverse soft-iron transformation matrix

    """

    hard_iron: NDArray
    inv_soft_iron: NDArray


@dataclass
class MleFitting(FittingAlgorithm):
    """Maximum Likelihood Estimation fitting for magnetometer calibration.

    Attributes
    ----------
    n: int
        Number of data points
    data : np.ndarray (n,3)
        Raw magnetometer readings
    transform : np.ndarray (3,3)
        Current transformation matrix (soft-iron correction)
    damping : float
        Initial damping factor for the optimization
    tol : float
        Tolerance for convergence
    max_iter : int
        Maximum number of iterations for optimization
    last_cost : float or None
        Cost from the last iteration, used for adaptive damping

    based on the paper "Geometric Approach to Strapdown Magnetometer Calibration in Sensor Frame" by
    J. F. Vasconcelos, G. Elkaim, C. Silvestre, P. Oliveira and B. Cardeira.

    """

    n: int
    data: NDArray
    transform: NDArray
    damping: float
    tol: float
    max_iter: int
    last_cost: float | None = None

    def __init__(
        self,
        data: NDArray,
        max_iter: int = 50,
        tol: float = 1e-10,
        damping: float = 1e-3,
    ) -> None:
        """Initialize the MLE fitting with the given data and parameters and perform the fitting."""
        self.data = data
        self.max_iter = max_iter
        self.tol = tol
        self.damping = damping
        self._vasconcelos()

    def _vasconcelos(self) -> None:
        """Estimate the ellipsoid parameters with Maximum Likelihood Estimator approach."""
        self._initialize()

        # Iteratively optimize using a damped Gauss-Newton method
        for _it in range(self.max_iter):
            stop = self._step()
            if stop:
                break

        self._calculate_inv_soft_iron()

    def _initialize(self) -> None:
        """Initialize the parameters."""
        self.n = self.data.shape[0]
        self.hard_iron = self.data.mean(axis=0)
        # Cov-based inverse sqrt (rough ellipsoid -> sphere mapping)
        C = np.cov((self.data - self.hard_iron).T)
        w, V = np.linalg.eigh(C)
        w = np.maximum(w, 1e-12)
        inv_sqrt_C = V @ np.diag(1.0 / np.sqrt(w)) @ V.T
        # For uniform points on unit sphere, Cov ~= (1/3) I, so scale by 1/sqrt(3)
        self.transform = (MAGNETIC_FIELD_STRENGTH / np.sqrt(3.0)) * inv_sqrt_C

    def _step(self) -> bool:
        """Perform a single optimization step of the MLE fitting.

        :return: True if the convergence criteria were met, False otherwise
        """
        U = self.data - self.hard_iron  # (n,3)
        Vv = (self.transform @ U.T).T  # (n,3) = T u_i
        norms = np.maximum(np.linalg.norm(Vv, axis=1), 1e-12)
        r = norms - MAGNETIC_FIELD_STRENGTH  # residuals

        cost = float(np.dot(r, r))

        # Build Jacobian J: (n,12) for [vec(T); b]
        # dr_i/dT = vec( a_i u_i^T ) where a_i = (T u_i)/||T u_i||
        # dr_i/db = -T^T a_i
        J = np.zeros((self.n, 12), dtype=float)
        for i in range(self.n):
            a = Vv[i] / norms[i]  # (3,)
            dT = np.outer(a, U[i])  # (3,3)
            J[i, :9] = dT.reshape(-1, order="F")  # vec in Fortran order
            J[i, 9:] = -(self.transform.T @ a)
        # Damped Gauss-Newton step: (J^T J + lam I) dx = -J^T r
        JTJ = J.T @ J
        g = J.T @ r
        A = JTJ + self.damping * np.eye(12)
        try:
            dx = np.linalg.solve(A, -g)
            fitting_success = self._evaluate_and_update_cost(cost=cost, dx=dx)

        except np.linalg.LinAlgError:
            # Increase damping if near-singular
            self.damping *= 10.0
            fitting_success = False

        return fitting_success

    def _evaluate_and_update_cost(self, cost: float, dx: NDArray) -> bool:
        """Update parameters based on the optimization step.

        :param cost: Current cost value
        :param dx: Parameter update vector
        :return: True if convergence criteria met, False otherwise
        """
        dT = dx[:9].reshape(3, 3, order="F")
        db = dx[9:]
        T_new = self.transform + dT
        b_new = self.hard_iron + db

        # Evaluate trial cost (simple acceptance with damping adjustment)
        U_new = self.data - b_new
        V_new = (T_new @ U_new.T).T
        norms_new = np.linalg.norm(V_new, axis=1)
        norms_new = np.maximum(norms_new, 1e-12)
        r_new = norms_new - MAGNETIC_FIELD_STRENGTH
        cost_new = float(np.dot(r_new, r_new))

        if self.last_cost is None:
            self.last_cost = cost

        if cost_new < cost:
            # Accept and reduce damping
            self.transform, self.hard_iron = T_new, b_new
            self.damping = max(self.damping / 3.0, 1e-12)
            self.last_cost = cost_new
            if np.linalg.norm(dx) < self.tol:
                return True
        else:
            # Reject and increase damping
            self.damping *= 10.0
            return False

        return False

    def _calculate_inv_soft_iron(self) -> None:
        """Calculate the inverse soft-iron matrix A_1 from the transformation T."""
        # --- Proposition 2: SVD(T*) -> (R_L, S_L, b) and then A_1 = S_L^{-1} R_L^T ---
        # T* = U S V^T, with S diagonal (positive)
        U_svd, s_vals, Vt_svd = np.linalg.svd(self.transform)
        V_svd = Vt_svd.T

        # Enforce V in SO(3) (det +1) as in the paper's convention
        if np.linalg.det(V_svd) < 0:
            V_svd[:, -1] *= -1.0
            U_svd[:, -1] *= -1.0
            # s_vals remain positive

        # From Proposition 2: R_L = V, S_L = S^{-1}, b = b_T
        # Therefore A_1 = S_L^{-1} R_L^T = S * V^T
        self.inv_soft_iron = np.diag(s_vals) @ V_svd.T


@dataclass
class LsFitting(FittingAlgorithm):
    """Least Squares fitting for magnetometer calibration."""

    def __init__(self, data: NDArray):
        """Initialize the LS fitting with the given data and perform the fitting."""
        self._li_griffiths(data)

    def _li_griffiths(self, data: NDArray) -> None:
        """Estimate ellipsoid parameters using the least squares method (one-step fitting).

        :param data: Numpy array of shape (N, 3) with magnetometer readings

        Note: copied from https://github.com/nliaudat/magnetometer_calibration/
        """
        s = data.T
        # Xi = (xi^2, yi^2, zi^2,2yizi, 2xizi, 2xiyi, 2xi, 2yi, 2zi,1).T

        # D is a design matrix of size (10, n)
        D = np.array(
            [
                s[0] ** 2.0,
                s[1] ** 2.0,
                s[2] ** 2.0,
                2.0 * s[1] * s[2],
                2.0 * s[0] * s[2],
                2.0 * s[0] * s[1],
                2.0 * s[0],
                2.0 * s[1],
                2.0 * s[2],
                np.ones_like(s[0]),
            ]
        )

        # S, S_11, S_12, S_21, S_22 (eq. 11)
        S = np.dot(D, D.T)
        S_11 = S[:6, :6]
        S_12 = S[:6, 6:]
        S_21 = S[6:, :6]
        S_22 = S[6:, 6:]

        # C (Eq. 8, k=4)
        C = np.array(
            [
                [-1, 1, 1, 0, 0, 0],
                [1, -1, 1, 0, 0, 0],
                [1, 1, -1, 0, 0, 0],
                [0, 0, 0, -4, 0, 0],
                [0, 0, 0, 0, -4, 0],
                [0, 0, 0, 0, 0, -4],
            ]
        )

        # v_1 (eq. 15, solution)
        E = np.dot(linalg.inv(C), S_11 - np.dot(S_12, np.dot(linalg.inv(S_22), S_21)))

        E_w, E_v = np.linalg.eig(E)
        v_1 = E_v[:, np.argmax(E_w)]
        if v_1[0] < 0:
            v_1 = -v_1

        # v_2 (eq. 13, solution)
        v_2 = np.dot(np.dot(-linalg.inv(S_22), S_21), v_1)

        # quadratic-form parameters, parameters h and f swapped as per correction by Roger R on Teslabs page
        M = np.array(
            [
                [v_1[0], v_1[5], v_1[4]],
                [v_1[5], v_1[1], v_1[3]],
                [v_1[4], v_1[3], v_1[2]],
            ]
        )
        n = np.array([[v_2[0]], [v_2[1]], [v_2[2]]])
        d = v_2[3]

        M_1 = linalg.inv(M)

        self.hard_iron = -np.dot(M_1, n)
        self.hard_iron = self.hard_iron.reshape(3)
        self.inv_soft_iron = np.real(
            MAGNETIC_FIELD_STRENGTH
            / np.sqrt(np.dot(n.T, np.dot(M_1, n)) - d)
            * linalg.sqrtm(M)
        )
