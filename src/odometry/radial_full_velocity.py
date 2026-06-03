import numpy as np
 
 
def az_Vr_to_full_V(azimuth: np.ndarray, Vr: np.ndarray):
    """Восстановление (Vx, Vy, |V|, угол) из радиальных скоростей через псевдообратную матрицу."""
    X = np.array([np.sin(azimuth), -np.cos(azimuth)]).T
    b = Vr.reshape(-1, 1)
    V_est = np.linalg.pinv(X) @ b
 
    Vx = V_est[0, 0]
    Vy = V_est[1, 0]
    angle = np.arctan(Vx / Vy)
    V = np.sqrt(Vx**2 + Vy**2)
 
    return V, angle, Vx, Vy
 