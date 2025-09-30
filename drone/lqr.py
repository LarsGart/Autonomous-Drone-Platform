from typing import Tuple
import numpy as np


class LQR:
    '''Linear Quadratic Regulator utility for discrete-time system.

    Solves the discrete-time LQR problem:
        minimize sum x.T Q x + u.T R u
        x_{k+1} = A x_k + B u_k

    Returns gain K such that u = -K x.

    Uses an iterative discrete Riccati solver.
    '''

    @staticmethod
    def _solve_dare(A: np.ndarray, B: np.ndarray, Q: np.ndarray, R: np.ndarray, eps: float = 1e-8, max_iter: int = 500) -> np.ndarray:
        '''Solve discrete algebraic Riccati equation (DARE) by iteration.'''
        P = Q.copy()
        for i in range(max_iter):
            P_next = A.T @ P @ A - A.T @ P @ B @ np.linalg.inv(R + B.T @ P @ B) @ B.T @ P @ A + Q
            if np.max(np.abs(P_next - P)) < eps:
                return P_next
            P = P_next
        return P  # last iterate if no convergence

    @staticmethod
    def _solve_discrete_lqr(A: np.ndarray, B: np.ndarray, Q: np.ndarray, R: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        '''Solve discrete LQR gain K and solution P.'''
        P = LQR._solve_dare(A, B, Q, R)
        # K = (R + B^T P B)^{-1} B^T P A
        K = np.linalg.inv(R + B.T @ P @ B) @ (B.T @ P @ A)
        return K, P
