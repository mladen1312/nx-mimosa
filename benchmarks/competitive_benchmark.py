"""
═══════════════════════════════════════════════════════════════════════════════
NX-MIMOSA COMPETITIVE BENCHMARK — CORRECTED VERSION
═══════════════════════════════════════════════════════════════════════════════
Fair comparison: All trackers run forward, then NX-MIMOSA v3.1 applies 
offline RTS smoothing to the full trajectory for comparison.
═══════════════════════════════════════════════════════════════════════════════
"""

import numpy as np
from numpy.linalg import inv, det
from dataclasses import dataclass
from typing import List, Tuple
from enum import Enum
import warnings
warnings.filterwarnings('ignore')


class ScenarioType(Enum):
    MISSILE_TERMINAL = "Missile Terminal (Mach 4, 9g)"
    HYPERSONIC_GLIDE = "Hypersonic Glide (Mach 5, 2g)"
    SAM_ENGAGEMENT = "SAM Engagement (300m/s, 6g)"
    DOGFIGHT_BFM = "Dogfight BFM (250m/s, 8g)"
    CRUISE_MISSILE = "Cruise Missile (250m/s, 3g)"
    BALLISTIC_REENTRY = "Ballistic Reentry (Mach 7, 1g)"
    UAV_SWARM = "UAV Swarm (50m/s, 2g)"
    STEALTH_AIRCRAFT = "Stealth Aircraft (200m/s, 4g)"


@dataclass
class ScenarioConfig:
    name: str
    speed_mps: float
    max_g: float
    duration_s: float
    dt: float
    sigma_meas: float
    segments: List[Tuple[float, float, str]]


SCENARIOS = {
    ScenarioType.MISSILE_TERMINAL: ScenarioConfig("Missile Terminal", 1360, 9.0, 10, 0.02, 5.0,
        [(0, 3, 'straight'), (3, 7, 'turn'), (7, 10, 'straight')]),
    ScenarioType.HYPERSONIC_GLIDE: ScenarioConfig("Hypersonic Glide", 1700, 2.0, 25, 0.02, 10.0,
        [(0, 5, 'straight'), (5, 20, 'weave'), (20, 25, 'straight')]),
    ScenarioType.SAM_ENGAGEMENT: ScenarioConfig("SAM Engagement", 300, 6.0, 15, 0.05, 8.0,
        [(0, 3, 'straight'), (3, 6, 'turn'), (6, 9, 'turn_neg'), (9, 12, 'turn'), (12, 15, 'straight')]),
    ScenarioType.DOGFIGHT_BFM: ScenarioConfig("Dogfight BFM", 250, 8.0, 20, 0.02, 3.0,
        [(0, 4, 'straight'), (4, 8, 'turn'), (8, 12, 'turn_neg'), (12, 16, 'turn'), (16, 20, 'straight')]),
    ScenarioType.CRUISE_MISSILE: ScenarioConfig("Cruise Missile", 250, 3.0, 30, 0.1, 15.0,
        [(0, 10, 'straight'), (10, 15, 'pop-up'), (15, 20, 'dive'), (20, 25, 'straight'), (25, 30, 'terminal')]),
    ScenarioType.BALLISTIC_REENTRY: ScenarioConfig("Ballistic Reentry", 2380, 1.0, 60, 0.1, 50.0,
        [(0, 20, 'straight'), (20, 40, 'slight_turn'), (40, 60, 'straight')]),
    ScenarioType.UAV_SWARM: ScenarioConfig("UAV Swarm", 50, 2.0, 60, 0.1, 2.0,
        [(i*5, (i+1)*5, 'random') for i in range(12)]),
    ScenarioType.STEALTH_AIRCRAFT: ScenarioConfig("Stealth Aircraft", 200, 4.0, 30, 0.5, 25.0,
        [(0, 10, 'straight'), (10, 20, 'turn'), (20, 30, 'straight')]),
}


def generate_trajectory(config: ScenarioConfig, seed: int) -> Tuple[np.ndarray, np.ndarray]:
    """Generate trajectory and measurements."""
    np.random.seed(seed)
    T = int(config.duration_s / config.dt)
    x_true = np.zeros((T, 4))
    x_true[0] = [10000, 0, -config.speed_mps, 0]
    
    g = 9.81
    for k in range(1, T):
        t = k * config.dt
        a_lat = 0.0
        
        for seg_start, seg_end, seg_type in config.segments:
            if seg_start <= t < seg_end:
                if seg_type == 'turn':
                    a_lat = config.max_g * g
                elif seg_type == 'turn_neg':
                    a_lat = -config.max_g * g
                elif seg_type == 'weave':
                    a_lat = config.max_g * g * np.sin(2*np.pi*t/8)
                elif seg_type == 'pop-up':
                    a_lat = config.max_g * g * 0.5
                elif seg_type == 'dive':
                    a_lat = -config.max_g * g * 0.5
                elif seg_type == 'slight_turn':
                    a_lat = config.max_g * g * 0.3
                elif seg_type == 'random':
                    if k % 10 == 0:
                        a_lat = np.random.uniform(-1, 1) * config.max_g * g
                elif seg_type == 'terminal':
                    a_lat = config.max_g * g * np.sin(0.5 * t)
                break
        
        vx, vy = x_true[k-1, 2], x_true[k-1, 3]
        v_mag = max(np.sqrt(vx**2 + vy**2), 1e-6)
        u_lat = np.array([-vy, vx]) / v_mag
        
        ax, ay = a_lat * u_lat[0], a_lat * u_lat[1]
        dt = config.dt
        
        x_true[k, 0] = x_true[k-1, 0] + vx * dt + 0.5 * ax * dt**2
        x_true[k, 1] = x_true[k-1, 1] + vy * dt + 0.5 * ay * dt**2
        x_true[k, 2] = x_true[k-1, 2] + ax * dt
        x_true[k, 3] = x_true[k-1, 3] + ay * dt
        
        # Maintain speed
        v_curr = np.sqrt(x_true[k, 2]**2 + x_true[k, 3]**2)
        if v_curr > 0:
            scale = config.speed_mps / v_curr
            x_true[k, 2:4] *= scale
    
    z_meas = x_true[:, :2] + np.random.randn(T, 2) * config.sigma_meas
    return x_true, z_meas


# =============================================================================
# TRACKERS
# =============================================================================

class EKF_CV:
    def __init__(self, dt, sigma, q=1.0):
        self.dt, self.sigma, self.q = dt, sigma, q
        self.F = np.array([[1,0,dt,0],[0,1,0,dt],[0,0,1,0],[0,0,0,1]])
        self.Q = q * np.diag([dt**4/4, dt**4/4, dt**2, dt**2])
        self.H = np.array([[1,0,0,0],[0,1,0,0]])
        self.R = sigma**2 * np.eye(2)
        
    def initialize(self, z0, v0):
        self.x = np.array([z0[0], z0[1], v0[0], v0[1]])
        self.P = np.diag([100, 100, 500, 500])**2
        
    def update(self, z):
        xp = self.F @ self.x
        Pp = self.F @ self.P @ self.F.T + self.Q
        y = z - self.H @ xp
        S = self.H @ Pp @ self.H.T + self.R
        K = Pp @ self.H.T @ inv(S)
        self.x = xp + K @ y
        self.P = (np.eye(4) - K @ self.H) @ Pp
        return self.x.copy()


class EKF_CA:
    def __init__(self, dt, sigma, q=10.0):
        self.dt, self.sigma, self.q = dt, sigma, q
        dt2 = dt**2
        self.F = np.array([[1,0,dt,0,dt2/2,0],[0,1,0,dt,0,dt2/2],
                          [0,0,1,0,dt,0],[0,0,0,1,0,dt],[0,0,0,0,1,0],[0,0,0,0,0,1]])
        self.Q = q * np.diag([dt**4/4, dt**4/4, dt**2, dt**2, 1, 1])
        self.H = np.array([[1,0,0,0,0,0],[0,1,0,0,0,0]])
        self.R = sigma**2 * np.eye(2)
        
    def initialize(self, z0, v0):
        self.x = np.array([z0[0], z0[1], v0[0], v0[1], 0, 0])
        self.P = np.diag([100, 100, 500, 500, 100, 100])**2
        
    def update(self, z):
        xp = self.F @ self.x
        Pp = self.F @ self.P @ self.F.T + self.Q
        y = z - self.H @ xp
        S = self.H @ Pp @ self.H.T + self.R
        K = Pp @ self.H.T @ inv(S)
        self.x = xp + K @ y
        self.P = (np.eye(6) - K @ self.H) @ Pp
        return self.x[:4].copy()


class AlphaBetaGamma:
    def __init__(self, dt, sigma):
        self.dt = dt
        self.alpha, self.beta, self.gamma = 0.5, 0.4, 0.1
        
    def initialize(self, z0, v0):
        self.x = np.array([z0[0], z0[1], v0[0], v0[1], 0, 0])
        
    def update(self, z):
        dt = self.dt
        xp = self.x[0] + self.x[2]*dt + 0.5*self.x[4]*dt**2
        yp = self.x[1] + self.x[3]*dt + 0.5*self.x[5]*dt**2
        vxp = self.x[2] + self.x[4]*dt
        vyp = self.x[3] + self.x[5]*dt
        
        rx, ry = z[0] - xp, z[1] - yp
        self.x[0] = xp + self.alpha * rx
        self.x[1] = yp + self.alpha * ry
        self.x[2] = vxp + self.beta * rx / dt
        self.x[3] = vyp + self.beta * ry / dt
        self.x[4] += self.gamma * rx / (0.5 * dt**2)
        self.x[5] += self.gamma * ry / (0.5 * dt**2)
        return self.x[:4].copy()


class IMM_Forward:
    def __init__(self, dt, sigma, omega=0.15):
        self.dt, self.sigma, self.omega = dt, sigma, omega
        self.n_models = 3
        self._build()
        
    def _build(self):
        dt, omega = self.dt, self.omega
        self.F = [np.array([[1,0,dt,0],[0,1,0,dt],[0,0,1,0],[0,0,0,1]])]
        if omega != 0:
            s, c = np.sin(omega*dt), np.cos(omega*dt)
            self.F.append(np.array([[1,0,s/omega,-(1-c)/omega],[0,1,(1-c)/omega,s/omega],[0,0,c,-s],[0,0,s,c]]))
            s, c = np.sin(-omega*dt), np.cos(-omega*dt)
            self.F.append(np.array([[1,0,s/(-omega),-(1-c)/(-omega)],[0,1,(1-c)/(-omega),s/(-omega)],[0,0,c,-s],[0,0,s,c]]))
        else:
            self.F.extend([self.F[0].copy(), self.F[0].copy()])
        self.Q = [0.5*np.diag([dt**4/4,dt**4/4,dt**2,dt**2]), 
                  5.0*np.diag([dt**4/4,dt**4/4,dt**2,dt**2]),
                  5.0*np.diag([dt**4/4,dt**4/4,dt**2,dt**2])]
        self.H = np.array([[1,0,0,0],[0,1,0,0]])
        self.R = self.sigma**2 * np.eye(2)
        p = 0.95
        self.PI = np.array([[p,(1-p)/2,(1-p)/2],[(1-p)/2,p,(1-p)/2],[(1-p)/2,(1-p)/2,p]])
        
    def initialize(self, z0, v0):
        x0 = np.array([z0[0], z0[1], v0[0], v0[1]])
        P0 = np.diag([100, 100, 500, 500])**2
        self.x = [x0.copy() for _ in range(3)]
        self.P = [P0.copy() for _ in range(3)]
        self.mu = np.array([0.8, 0.1, 0.1])
        
    def update(self, z):
        n = 3
        c_bar = self.PI.T @ self.mu
        mu_ij = np.array([[self.PI[i,j]*self.mu[i]/(c_bar[j]+1e-10) for j in range(n)] for i in range(n)])
        
        x_mixed = [sum(mu_ij[i,j]*self.x[i] for i in range(n)) for j in range(n)]
        P_mixed = [sum(mu_ij[i,j]*(self.P[i]+np.outer(self.x[i]-x_mixed[j],self.x[i]-x_mixed[j])) for i in range(n)) for j in range(n)]
        
        x_filt, P_filt, liks = [], [], []
        for j in range(n):
            xp = self.F[j] @ x_mixed[j]
            Pp = self.F[j] @ P_mixed[j] @ self.F[j].T + self.Q[j]
            y = z - self.H @ xp
            S = self.H @ Pp @ self.H.T + self.R + 1e-6*np.eye(2)
            K = Pp @ self.H.T @ inv(S)
            x_filt.append(xp + K @ y)
            P_filt.append((np.eye(4) - K @ self.H) @ Pp)
            liks.append(max(np.exp(-0.5*y@inv(S)@y)/np.sqrt((2*np.pi)**2*det(S)), 1e-10))
        
        mu_unnorm = c_bar * np.array(liks)
        self.mu = mu_unnorm / (mu_unnorm.sum() + 1e-10)
        self.x, self.P = x_filt, P_filt
        return sum(self.mu[j] * x_filt[j] for j in range(n))


class NX_MIMOSA_V20:
    """NX-MIMOSA v2.0: IMM + Adaptive Q + VS-IMM (forward only)."""
    def __init__(self, dt, sigma, omega=0.15):
        self.dt, self.sigma, self.omega = dt, sigma, omega
        self.n_models = 3
        self._build()
        self.q_scale = 1.0
        
    def _build(self):
        dt, omega = self.dt, self.omega
        self.F = [np.array([[1,0,dt,0],[0,1,0,dt],[0,0,1,0],[0,0,0,1]])]
        if omega != 0:
            s, c = np.sin(omega*dt), np.cos(omega*dt)
            self.F.append(np.array([[1,0,s/omega,-(1-c)/omega],[0,1,(1-c)/omega,s/omega],[0,0,c,-s],[0,0,s,c]]))
            s, c = np.sin(-omega*dt), np.cos(-omega*dt)
            self.F.append(np.array([[1,0,s/(-omega),-(1-c)/(-omega)],[0,1,(1-c)/(-omega),s/(-omega)],[0,0,c,-s],[0,0,s,c]]))
        else:
            self.F.extend([self.F[0].copy(), self.F[0].copy()])
        self.Q_base = [0.5*np.diag([dt**4/4,dt**4/4,dt**2,dt**2]), 
                       5.0*np.diag([dt**4/4,dt**4/4,dt**2,dt**2]),
                       5.0*np.diag([dt**4/4,dt**4/4,dt**2,dt**2])]
        self.H = np.array([[1,0,0,0],[0,1,0,0]])
        self.R = self.sigma**2 * np.eye(2)
        
    def initialize(self, z0, v0):
        x0 = np.array([z0[0], z0[1], v0[0], v0[1]])
        P0 = np.diag([100, 100, 500, 500])**2
        self.x = [x0.copy() for _ in range(3)]
        self.P = [P0.copy() for _ in range(3)]
        self.mu = np.array([0.8, 0.1, 0.1])
        self.q_scale = 1.0
        
    def update(self, z):
        n = 3
        # VS-IMM: Dynamic TPM
        mu_cv = self.mu[0]
        p_stay = 0.98 if mu_cv > 0.9 else (0.95 if mu_cv > 0.7 else 0.90)
        PI = np.array([[p_stay,(1-p_stay)/2,(1-p_stay)/2],
                      [(1-p_stay)/2,p_stay,(1-p_stay)/2],
                      [(1-p_stay)/2,(1-p_stay)/2,p_stay]])
        
        c_bar = PI.T @ self.mu
        mu_ij = np.array([[PI[i,j]*self.mu[i]/(c_bar[j]+1e-10) for j in range(n)] for i in range(n)])
        
        x_mixed = [sum(mu_ij[i,j]*self.x[i] for i in range(n)) for j in range(n)]
        P_mixed = [sum(mu_ij[i,j]*(self.P[i]+np.outer(self.x[i]-x_mixed[j],self.x[i]-x_mixed[j])) for i in range(n)) for j in range(n)]
        
        Q = [self.q_scale * q for q in self.Q_base]
        
        x_filt, P_filt, liks = [], [], []
        total_nis = 0
        for j in range(n):
            xp = self.F[j] @ x_mixed[j]
            Pp = self.F[j] @ P_mixed[j] @ self.F[j].T + Q[j]
            y = z - self.H @ xp
            S = self.H @ Pp @ self.H.T + self.R + 1e-6*np.eye(2)
            S_inv = inv(S)
            K = Pp @ self.H.T @ S_inv
            nis_j = y @ S_inv @ y
            total_nis += self.mu[j] * nis_j
            
            x_f = xp + K @ y
            I_KH = np.eye(4) - K @ self.H
            P_f = I_KH @ Pp @ I_KH.T + K @ self.R @ K.T  # Joseph form
            x_filt.append(x_f)
            P_filt.append(P_f)
            liks.append(max(np.exp(-0.5*nis_j)/np.sqrt((2*np.pi)**2*det(S)), 1e-10))
        
        # Adaptive Q
        if total_nis > 5.991:
            self.q_scale = min(self.q_scale * 1.3, 3.0)
        elif total_nis < 2.996:
            self.q_scale = max(self.q_scale * 0.98, 0.3)
        
        mu_unnorm = c_bar * np.array(liks)
        self.mu = mu_unnorm / (mu_unnorm.sum() + 1e-10)
        self.x, self.P = x_filt, P_filt
        return sum(self.mu[j] * x_filt[j] for j in range(n))


class NX_MIMOSA_V31:
    """NX-MIMOSA v3.1: Full with offline RTS smoother."""
    def __init__(self, dt, sigma, omega=0.15):
        self.dt, self.sigma, self.omega = dt, sigma, omega
        self.n_models = 3
        self._build()
        self.q_scale = 1.0
        self.history = []
        
    def _build(self):
        dt, omega = self.dt, self.omega
        self.F = [np.array([[1,0,dt,0],[0,1,0,dt],[0,0,1,0],[0,0,0,1]])]
        if omega != 0:
            s, c = np.sin(omega*dt), np.cos(omega*dt)
            self.F.append(np.array([[1,0,s/omega,-(1-c)/omega],[0,1,(1-c)/omega,s/omega],[0,0,c,-s],[0,0,s,c]]))
            s, c = np.sin(-omega*dt), np.cos(-omega*dt)
            self.F.append(np.array([[1,0,s/(-omega),-(1-c)/(-omega)],[0,1,(1-c)/(-omega),s/(-omega)],[0,0,c,-s],[0,0,s,c]]))
        else:
            self.F.extend([self.F[0].copy(), self.F[0].copy()])
        self.Q_base = [0.5*np.diag([dt**4/4,dt**4/4,dt**2,dt**2]), 
                       5.0*np.diag([dt**4/4,dt**4/4,dt**2,dt**2]),
                       5.0*np.diag([dt**4/4,dt**4/4,dt**2,dt**2])]
        self.H = np.array([[1,0,0,0],[0,1,0,0]])
        self.R = self.sigma**2 * np.eye(2)
        
    def initialize(self, z0, v0):
        x0 = np.array([z0[0], z0[1], v0[0], v0[1]])
        P0 = np.diag([100, 100, 500, 500])**2
        self.x = [x0.copy() for _ in range(3)]
        self.P = [P0.copy() for _ in range(3)]
        self.mu = np.array([0.8, 0.1, 0.1])
        self.q_scale = 1.0
        self.history = []
        
    def update(self, z):
        n = 3
        mu_cv = self.mu[0]
        p_stay = 0.98 if mu_cv > 0.9 else (0.95 if mu_cv > 0.7 else 0.90)
        PI = np.array([[p_stay,(1-p_stay)/2,(1-p_stay)/2],
                      [(1-p_stay)/2,p_stay,(1-p_stay)/2],
                      [(1-p_stay)/2,(1-p_stay)/2,p_stay]])
        
        c_bar = PI.T @ self.mu
        mu_ij = np.array([[PI[i,j]*self.mu[i]/(c_bar[j]+1e-10) for j in range(n)] for i in range(n)])
        
        x_mixed = [sum(mu_ij[i,j]*self.x[i] for i in range(n)) for j in range(n)]
        P_mixed = [sum(mu_ij[i,j]*(self.P[i]+np.outer(self.x[i]-x_mixed[j],self.x[i]-x_mixed[j])) for i in range(n)) for j in range(n)]
        
        Q = [self.q_scale * q for q in self.Q_base]
        
        x_filt, P_filt, x_pred, P_pred, liks = [], [], [], [], []
        total_nis = 0
        for j in range(n):
            xp = self.F[j] @ x_mixed[j]  # BUGFIX: F @ x_mixed
            Pp = self.F[j] @ P_mixed[j] @ self.F[j].T + Q[j]
            x_pred.append(xp.copy())
            P_pred.append(Pp.copy())
            
            y = z - self.H @ xp
            S = self.H @ Pp @ self.H.T + self.R + 1e-6*np.eye(2)
            S_inv = inv(S)
            K = Pp @ self.H.T @ S_inv
            nis_j = y @ S_inv @ y
            total_nis += self.mu[j] * nis_j
            
            x_f = xp + K @ y
            I_KH = np.eye(4) - K @ self.H
            P_f = I_KH @ Pp @ I_KH.T + K @ self.R @ K.T
            x_filt.append(x_f)
            P_filt.append(P_f)
            liks.append(max(np.exp(-0.5*nis_j)/np.sqrt((2*np.pi)**2*det(S)), 1e-10))
        
        if total_nis > 5.991:
            self.q_scale = min(self.q_scale * 1.3, 3.0)
        elif total_nis < 2.996:
            self.q_scale = max(self.q_scale * 0.98, 0.3)
        
        mu_unnorm = c_bar * np.array(liks)
        self.mu = mu_unnorm / (mu_unnorm.sum() + 1e-10)
        
        # Store for smoother (BUGFIX: store x_pred = F @ x_mixed)
        self.history.append({
            'x_filt': [x.copy() for x in x_filt],
            'P_filt': [P.copy() for P in P_filt],
            'x_pred': [x.copy() for x in x_pred],
            'P_pred': [P.copy() for P in P_pred],
            'mu': self.mu.copy(),
            'F': [F.copy() for F in self.F]
        })
        
        self.x, self.P = x_filt, P_filt
        return sum(self.mu[j] * x_filt[j] for j in range(n))
    
    def smooth(self) -> np.ndarray:
        """Offline RTS smoother over full trajectory."""
        if len(self.history) < 2:
            return np.array([sum(self.history[0]['mu'][j] * self.history[0]['x_filt'][j] for j in range(3))])
        
        n = 3
        L = len(self.history)
        
        # Initialize
        x_smooth = [[h['x_filt'][j].copy() for j in range(n)] for h in self.history]
        P_smooth = [[h['P_filt'][j].copy() for j in range(n)] for h in self.history]
        
        # Backward pass
        for k in range(L - 2, -1, -1):
            h = self.history[k]
            h_next = self.history[k + 1]
            
            for j in range(n):
                F = h['F'][j]
                x_f = h['x_filt'][j]
                P_f = h['P_filt'][j]
                x_p = h_next['x_pred'][j]  # BUGFIX: This is F @ x_mixed
                P_p = h_next['P_pred'][j]
                
                try:
                    G = P_f @ F.T @ inv(P_p + 1e-6*np.eye(4))
                except:
                    G = np.zeros((4, 4))
                
                x_smooth[k][j] = x_f + G @ (x_smooth[k+1][j] - x_p)
                P_smooth[k][j] = P_f + G @ (P_smooth[k+1][j] - P_p) @ G.T
        
        # Combine
        result = np.zeros((L, 4))
        for k in range(L):
            mu = self.history[k]['mu']
            result[k] = sum(mu[j] * x_smooth[k][j] for j in range(n))
        
        return result


# =============================================================================
# BENCHMARK
# =============================================================================

def run_benchmark(n_runs: int = 30):
    print("=" * 90)
    print("NX-MIMOSA COMPETITIVE BENCHMARK — CORRECTED")
    print("=" * 90)
    print(f"\nMonte Carlo: {n_runs} runs per scenario")
    print("Comparison: Forward trackers vs v3.1 with OFFLINE smoother\n")
    
    trackers_forward = [
        ('EKF-CV', EKF_CV),
        ('EKF-CA', EKF_CA),
        ('α-β-γ', AlphaBetaGamma),
        ('IMM-Forward', IMM_Forward),
        ('NX-MIMOSA v2.0', NX_MIMOSA_V20),
    ]
    
    results = {s: {n: [] for n, _ in trackers_forward} for s in ScenarioType}
    results_v31_forward = {s: [] for s in ScenarioType}
    results_v31_smooth = {s: [] for s in ScenarioType}
    
    for scenario_type, config in SCENARIOS.items():
        print(f"{'─'*90}")
        print(f"SCENARIO: {config.name} | Speed: {config.speed_mps:.0f} m/s, Max-g: {config.max_g}")
        
        omega = config.max_g * 9.81 / max(config.speed_mps, 1)
        
        for run in range(n_runs):
            x_true, z_meas = generate_trajectory(config, seed=run*1000 + hash(scenario_type.name) % 1000)
            v_init = np.array([x_true[0, 2], x_true[0, 3]])
            
            # Run forward trackers
            for name, TrackerClass in trackers_forward:
                if name in ['EKF-CV', 'EKF-CA', 'α-β-γ']:
                    tracker = TrackerClass(config.dt, config.sigma_meas)
                else:
                    tracker = TrackerClass(config.dt, config.sigma_meas, omega)
                tracker.initialize(z_meas[0], v_init)
                
                ests = [tracker.update(z_meas[k]) for k in range(1, len(z_meas))]
                ests = np.array(ests)
                rmse = np.sqrt(np.mean((ests[:, :2] - x_true[1:, :2])**2))
                results[scenario_type][name].append(rmse)
            
            # Run v3.1 (forward + smoother)
            tracker_v31 = NX_MIMOSA_V31(config.dt, config.sigma_meas, omega)
            tracker_v31.initialize(z_meas[0], v_init)
            
            ests_forward = [tracker_v31.update(z_meas[k]) for k in range(1, len(z_meas))]
            ests_forward = np.array(ests_forward)
            rmse_forward = np.sqrt(np.mean((ests_forward[:, :2] - x_true[1:, :2])**2))
            results_v31_forward[scenario_type].append(rmse_forward)
            
            # Offline smoother
            ests_smooth = tracker_v31.smooth()
            rmse_smooth = np.sqrt(np.mean((ests_smooth[:, :2] - x_true[1:, :2])**2))
            results_v31_smooth[scenario_type].append(rmse_smooth)
        
        print(f"  ✓ Complete")
    
    # Print results
    print("\n" + "=" * 120)
    print("BENCHMARK RESULTS — POSITION RMSE (meters)")
    print("=" * 120)
    
    header = f"{'Scenario':<25}"
    for name, _ in trackers_forward:
        header += f" {name:>14}"
    header += f" {'v3.1-Forward':>14} {'v3.1-Smooth':>14}"
    print(header)
    print("-" * 120)
    
    wins_v31 = 0
    total = 0
    
    for scenario_type in ScenarioType:
        config = SCENARIOS[scenario_type]
        row = f"{config.name[:24]:<25}"
        
        all_rmse = []
        for name, _ in trackers_forward:
            mean_rmse = np.mean(results[scenario_type][name])
            row += f" {mean_rmse:>14.2f}"
            all_rmse.append((name, mean_rmse))
        
        mean_v31_fwd = np.mean(results_v31_forward[scenario_type])
        mean_v31_smooth = np.mean(results_v31_smooth[scenario_type])
        row += f" {mean_v31_fwd:>14.2f} {mean_v31_smooth:>14.2f}"
        
        all_rmse.append(('v3.1-Forward', mean_v31_fwd))
        all_rmse.append(('v3.1-Smooth', mean_v31_smooth))
        
        # Check winner
        sorted_rmse = sorted(all_rmse, key=lambda x: x[1])
        if sorted_rmse[0][0] == 'v3.1-Smooth':
            wins_v31 += 1
            row += " 🥇"
        total += 1
        
        print(row)
    
    print("-" * 120)
    
    # Averages
    avg_row = f"{'AVERAGE':<25}"
    for name, _ in trackers_forward:
        avg = np.mean([np.mean(results[s][name]) for s in ScenarioType])
        avg_row += f" {avg:>14.2f}"
    
    avg_v31_fwd = np.mean([np.mean(results_v31_forward[s]) for s in ScenarioType])
    avg_v31_smooth = np.mean([np.mean(results_v31_smooth[s]) for s in ScenarioType])
    avg_row += f" {avg_v31_fwd:>14.2f} {avg_v31_smooth:>14.2f}"
    print(avg_row)
    print("=" * 120)
    
    # Improvement analysis
    print("\n" + "=" * 90)
    print("IMPROVEMENT ANALYSIS (v3.1 Smoother vs competitors)")
    print("=" * 90)
    
    for name, _ in trackers_forward:
        avg_comp = np.mean([np.mean(results[s][name]) for s in ScenarioType])
        improvement = (avg_comp - avg_v31_smooth) / avg_comp * 100
        print(f"  vs {name:<20}: {improvement:>+6.1f}%")
    
    print(f"\n  vs v3.1-Forward:        {(avg_v31_fwd - avg_v31_smooth) / avg_v31_fwd * 100:>+6.1f}%")
    
    print(f"\n🏆 v3.1 Smoother WINS: {wins_v31}/{total} scenarios")
    
    return results, results_v31_smooth


if __name__ == "__main__":
    run_benchmark(n_runs=30)
