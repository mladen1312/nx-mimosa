#!/usr/bin/env python3
"""
NX-MIMOSA v3.2 UPGRADE + BENCHMARK RERUN
Changes: 4-model IMM, aggressive adaptive Q, NIS EMA, cov-weighted smoother
"""
import numpy as np
from numpy.linalg import inv
from scipy.linalg import block_diag
import time, warnings
warnings.filterwarnings('ignore')

import sys; sys.path.insert(0, '/home/claude')
from benchmark_vs_goldstandard import (
    get_scenarios, compute_pos_rmse, compute_vel_rmse,
    cv_matrices, ct_matrices, measurement_matrix, measurement_noise,
    run_filterpy_imm, run_stonesoup_ekf_rts, NxMimosaV3
)

class NxMimosaV32:
    """NX-MIMOSA v3.2: 4-Model IMM + Enhanced Adaptation + Cov-weighted Smoother."""
    
    def __init__(self, dt, sigma_meas, q_base):
        self.dt = dt
        self.H = measurement_matrix()
        self.R = measurement_noise(sigma_meas)
        self.q_base = q_base
        self.n_models = 4  # CV, CT+, CT-, CV-HIGH-Q
        self.omegas = [0.0, 0.1, -0.1, 0.0]
        self.q_multipliers = [1.0, 1.0, 1.0, 5.0]
        self.mu = np.array([0.6, 0.1, 0.1, 0.2])
        self.TPM_base = np.array([
            [0.90, 0.025, 0.025, 0.05],
            [0.05, 0.85,  0.05,  0.05],
            [0.05, 0.05,  0.85,  0.05],
            [0.10, 0.05,  0.05,  0.80],
        ])
        self.nis_ema_alpha = 0.15
        
    def _get_model_FQ(self, j, q_scale=1.0):
        q_eff = self.q_base * self.q_multipliers[j] * q_scale
        return ct_matrices(self.dt, q_eff, self.omegas[j])
    
    def _adaptive_q_scale(self, innov, S, nis_ema):
        nis = float(innov @ inv(S) @ innov)
        expected = len(innov)
        ratio = max(nis, nis_ema) / expected
        if ratio > 5.0: return min(ratio * 1.5, 25.0)
        elif ratio > 2.0: return ratio
        elif ratio < 0.2: return max(ratio, 0.05)
        return 1.0
    
    def _vs_tpm(self, mu, nis_ema_global):
        expected = 2.0
        if nis_ema_global > 3 * expected:
            TPM = self.TPM_base.copy()
            for i in range(4):
                if i != 3:
                    TPM[i, 3] += 0.05
                    TPM[i, i] -= 0.05
            return TPM
        max_prob = np.max(mu)
        if max_prob > 0.90: p_stay = 0.97
        elif max_prob > 0.70: p_stay = 0.93
        else: p_stay = 0.88
        TPM = np.full((4, 4), (1 - p_stay) / 3)
        np.fill_diagonal(TPM, p_stay)
        return TPM
    
    def run(self, measurements):
        N = len(measurements); nx = 4; nm = self.n_models
        x_filt = np.zeros((nm, N, nx)); P_filt = np.zeros((nm, N, nx, nx))
        x_pred = np.zeros((nm, N, nx)); P_pred = np.zeros((nm, N, nx, nx))
        mu_hist = np.zeros((N, nm))
        x_combined = np.zeros((N, nx)); P_combined = np.zeros((N, nx, nx))
        
        x0 = np.array([measurements[0,0], 0.0, measurements[0,1], 0.0])
        P0 = np.diag([self.R[0,0], 100.0, self.R[1,1], 100.0])
        for j in range(nm):
            x_filt[j,0]=x0.copy(); P_filt[j,0]=P0.copy()
            x_pred[j,0]=x0.copy(); P_pred[j,0]=P0.copy()
        mu = self.mu.copy(); mu_hist[0]=mu
        x_combined[0]=x0; P_combined[0]=P0
        nis_ema = np.ones(nm)*2.0; nis_ema_global = 2.0
        
        for k in range(1, N):
            z = measurements[k]; TPM = self._vs_tpm(mu, nis_ema_global)
            c_bar = TPM.T @ mu
            mu_mix = np.zeros((nm,nm))
            for i in range(nm):
                for j in range(nm):
                    mu_mix[i,j] = TPM[i,j]*mu[i]/max(c_bar[j],1e-30)
            x_mix = np.zeros((nm,nx)); P_mix = np.zeros((nm,nx,nx))
            for j in range(nm):
                for i in range(nm): x_mix[j] += mu_mix[i,j]*x_filt[i,k-1]
                for i in range(nm):
                    d = x_filt[i,k-1]-x_mix[j]
                    P_mix[j] += mu_mix[i,j]*(P_filt[i,k-1]+np.outer(d,d))
            
            likelihoods = np.zeros(nm)
            for j in range(nm):
                Fj, Qj = self._get_model_FQ(j)
                xp = Fj@x_mix[j]; Pp = Fj@P_mix[j]@Fj.T + Qj
                innov = z - self.H@xp; S = self.H@Pp@self.H.T + self.R
                qs = self._adaptive_q_scale(innov, S, nis_ema[j])
                if abs(qs-1.0) > 0.01:
                    _, Qa = self._get_model_FQ(j, qs)
                    Pp = Fj@P_mix[j]@Fj.T + Qa
                    S = self.H@Pp@self.H.T + self.R
                x_pred[j,k]=xp; P_pred[j,k]=Pp
                Si = inv(S); K = Pp@self.H.T@Si
                nis_k = float(innov@Si@innov)
                nis_ema[j] = self.nis_ema_alpha*nis_k + (1-self.nis_ema_alpha)*nis_ema[j]
                x_filt[j,k] = xp + K@innov
                IKH = np.eye(nx)-K@self.H
                P_filt[j,k] = IKH@Pp@IKH.T + K@self.R@K.T
                _,logdet = np.linalg.slogdet(S)
                likelihoods[j] = np.exp(np.clip(-0.5*(nis_k+logdet+2*np.log(2*np.pi)),-500,0))
            
            nis_ema_global = self.nis_ema_alpha*np.mean(nis_ema)+(1-self.nis_ema_alpha)*nis_ema_global
            mu = c_bar*likelihoods
            ms = np.sum(mu)
            mu = mu/ms if ms>1e-300 else np.ones(nm)/nm
            mu_hist[k] = mu
            x_combined[k] = sum(mu[j]*x_filt[j,k] for j in range(nm))
            for j in range(nm):
                d = x_filt[j,k]-x_combined[k]
                P_combined[k] += mu[j]*(P_filt[j,k]+np.outer(d,d))
        
        forward_states = x_combined.copy()
        
        # Per-model RTS smoother
        x_sm_j = np.zeros((nm,N,nx)); P_sm_j = np.zeros((nm,N,nx,nx))
        for j in range(nm):
            x_sm_j[j,N-1]=x_filt[j,N-1].copy(); P_sm_j[j,N-1]=P_filt[j,N-1].copy()
            for k in range(N-2,-1,-1):
                Fj,_ = self._get_model_FQ(j)
                G = P_filt[j,k]@Fj.T@inv(P_pred[j,k+1]+np.eye(nx)*1e-10)
                x_sm_j[j,k] = x_filt[j,k]+G@(x_sm_j[j,k+1]-x_pred[j,k+1])
                P_sm_j[j,k] = P_filt[j,k]+G@(P_sm_j[j,k+1]-P_pred[j,k+1])@G.T
                P_sm_j[j,k] = 0.5*(P_sm_j[j,k]+P_sm_j[j,k].T)
        
        # Covariance-weighted combination
        x_smooth = np.zeros((N,nx)); P_smooth = np.zeros((N,nx,nx))
        for k in range(N):
            w = np.array([mu_hist[k,j]/(np.trace(P_sm_j[j,k])+1e-10) for j in range(nm)])
            w /= w.sum()+1e-30
            for j in range(nm): x_smooth[k] += w[j]*x_sm_j[j,k]
            for j in range(nm):
                d = x_sm_j[j,k]-x_smooth[k]
                P_smooth[k] += w[j]*(P_sm_j[j,k]+np.outer(d,d))
        
        return x_smooth, P_smooth, forward_states


def run_comparison(n_runs=30, seed=42):
    scenarios = get_scenarios()
    print("="*110)
    print("NX-MIMOSA v3.2 UPGRADE BENCHMARK")
    print(f"MC runs: {n_runs} | Seed: {seed}")
    print("="*110)
    print("\nv3.1→v3.2: +4th model(CV-HIGH-Q) +Aggressive adaptive Q(25×) +NIS EMA +Cov-weighted smoother\n")
    
    all_rmse = {k:[] for k in ["StoneSoup EKF+RTS","FilterPy IMM",
                "NX-MIMOSA v3.1(s)","NX-MIMOSA v3.2(s)","NX-MIMOSA v3.1(f)","NX-MIMOSA v3.2(f)"]}
    
    for si, sc in enumerate(scenarios):
        print(f"{'─'*110}")
        print(f"SCENARIO {si+1}/8: {sc.name} (dt={sc.dt}, σ={sc.sigma_pos}, q={sc.sigma_vel})")
        truth, _ = sc.generate_truth()
        accum = {k:[] for k in all_rmse}
        
        for run in range(n_runs):
            rng = np.random.default_rng(seed*1000+si*100+run)
            meas = sc.generate_measurements(truth, rng)
            
            try:
                est_ss = run_stonesoup_ekf_rts(meas, sc.dt, sc.sigma_pos, sc.sigma_vel)
                accum["StoneSoup EKF+RTS"].append(compute_pos_rmse(est_ss, truth))
            except: accum["StoneSoup EKF+RTS"].append(float('nan'))
            
            est_fp = run_filterpy_imm(meas, sc.dt, sc.sigma_pos, sc.sigma_vel)
            accum["FilterPy IMM"].append(compute_pos_rmse(est_fp, truth))
            
            m31 = NxMimosaV3(sc.dt, sc.sigma_pos, sc.sigma_vel)
            s31,_,f31 = m31.run(meas)
            accum["NX-MIMOSA v3.1(s)"].append(compute_pos_rmse(s31, truth))
            accum["NX-MIMOSA v3.1(f)"].append(compute_pos_rmse(f31, truth))
            
            m32 = NxMimosaV32(sc.dt, sc.sigma_pos, sc.sigma_vel)
            s32,_,f32 = m32.run(meas)
            accum["NX-MIMOSA v3.2(s)"].append(compute_pos_rmse(s32, truth))
            accum["NX-MIMOSA v3.2(f)"].append(compute_pos_rmse(f32, truth))
        
        print(f"  {'Algorithm':<30} {'RMSE(m)':>10} {'vs v3.1':>10}")
        print(f"  {'─'*52}")
        key_algos = ["StoneSoup EKF+RTS","FilterPy IMM","NX-MIMOSA v3.1(s)","NX-MIMOSA v3.2(s)"]
        vals = {k: np.nanmean(accum[k]) for k in key_algos}
        for rank,(name,v) in enumerate(sorted(vals.items(), key=lambda x:x[1])):
            medal = ["🥇","🥈","🥉","  "][rank]
            delta = ""
            if "v3.2" in name:
                v31 = vals["NX-MIMOSA v3.1(s)"]
                d = (v31-v)/v31*100
                delta = f"  {'+' if d>0 else ''}{d:.1f}%"
            print(f"  {medal} {name:<28} {v:>8.2f} m{delta}")
        
        for k in all_rmse: all_rmse[k].append(np.nanmean(accum[k]))
    
    # Grand summary
    print(f"\n{'='*110}")
    print("GRAND SUMMARY")
    print(f"{'='*110}")
    
    print(f"\n{'Scenario':<38} {'SS EKF+RTS':>12} {'FP IMM':>12} {'v3.1(s)':>12} {'v3.2(s)':>12} {'v3.2 delta':>12} {'Winner':>15}")
    print("─"*115)
    
    wins32 = 0
    for i,sc in enumerate(scenarios):
        sn = sc.name[:36]
        ss = all_rmse["StoneSoup EKF+RTS"][i]
        fp = all_rmse["FilterPy IMM"][i]
        m31 = all_rmse["NX-MIMOSA v3.1(s)"][i]
        m32 = all_rmse["NX-MIMOSA v3.2(s)"][i]
        d = (m31-m32)/m31*100
        best_val = min(ss,fp,m31,m32)
        winner = "v3.2" if m32==best_val else "v3.1" if m31==best_val else "StoneSoup" if ss==best_val else "FilterPy"
        if m32 <= min(ss,fp,m31): wins32 += 1
        print(f"{sn:<38} {ss:>10.2f}m {fp:>10.2f}m {m31:>10.2f}m {m32:>10.2f}m {d:>+10.1f}% {winner:>15}")
    
    print("─"*115)
    ga = {k: np.nanmean(v) for k,v in all_rmse.items()}
    print(f"{'GRAND AVERAGE':<38} {ga['StoneSoup EKF+RTS']:>10.2f}m {ga['FilterPy IMM']:>10.2f}m {ga['NX-MIMOSA v3.1(s)']:>10.2f}m {ga['NX-MIMOSA v3.2(s)']:>10.2f}m {(ga['NX-MIMOSA v3.1(s)']-ga['NX-MIMOSA v3.2(s)'])/ga['NX-MIMOSA v3.1(s)']*100:>+10.1f}%")
    
    print(f"\n  v3.2 WINS: {wins32}/8 scenarios")
    print(f"  v3.1→v3.2 improvement: {(ga['NX-MIMOSA v3.1(s)']-ga['NX-MIMOSA v3.2(s)'])/ga['NX-MIMOSA v3.1(s)']*100:+.1f}%")
    print(f"  v3.2 vs StoneSoup: {(ga['StoneSoup EKF+RTS']-ga['NX-MIMOSA v3.2(s)'])/ga['StoneSoup EKF+RTS']*100:+.1f}%")
    
    # Forward-only comparison
    print(f"\n  FORWARD-ONLY (real-time capable):")
    print(f"    FilterPy IMM:        {ga['FilterPy IMM']:.2f} m")
    print(f"    NX-MIMOSA v3.1(fwd): {ga['NX-MIMOSA v3.1(f)']:.2f} m")
    print(f"    NX-MIMOSA v3.2(fwd): {ga['NX-MIMOSA v3.2(f)']:.2f} m")
    imp_fwd = (ga['FilterPy IMM']-ga['NX-MIMOSA v3.2(f)'])/ga['FilterPy IMM']*100
    print(f"    v3.2(fwd) vs FilterPy IMM: +{imp_fwd:.1f}%")
    
    return all_rmse

if __name__ == "__main__":
    run_comparison(n_runs=30, seed=42)
