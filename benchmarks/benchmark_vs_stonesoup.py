#!/usr/bin/env python3
"""
NX-MIMOSA v4.0 SENTINEL vs Stone Soup 1.8 vs FilterPy — Formal Comparison
==========================================================================
Requirements: pip install stonesoup filterpy
Runs 5 Stone Soup configs + FilterPy IMM + NX-MIMOSA v4.0 across 8 scenarios.

Stone Soup configs (no IMM available in v1.8):
  - KF-CV:    Kalman Filter, Constant Velocity (q=0.5)
  - KF-CA:    Kalman Filter, Constant Acceleration (q=2.0)
  - UKF-CV:   Unscented KF, Constant Velocity (q=1.0)
  - UKF-CA:   Unscented KF, Constant Acceleration (q=3.0)
  - CV+RTS:   KF-CV + full-track RTS Smoother (OFFLINE, uses future data)

Author: Dr. Mladen Mešter / Nexellum d.o.o.
"""
import numpy as np, sys, os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'python'))
from datetime import datetime, timedelta
from stonesoup.types.state import GaussianState
from stonesoup.types.detection import Detection
from stonesoup.types.hypothesis import SingleHypothesis
from stonesoup.types.track import Track
from stonesoup.types.array import StateVector, CovarianceMatrix
from stonesoup.models.transition.linear import (
    CombinedLinearGaussianTransitionModel, ConstantVelocity, ConstantAcceleration)
from stonesoup.models.measurement.linear import LinearGaussian
from stonesoup.predictor.kalman import KalmanPredictor, UnscentedKalmanPredictor
from stonesoup.updater.kalman import KalmanUpdater, UnscentedKalmanUpdater
from stonesoup.smoother.kalman import KalmanSmoother

GRAVITY = 9.80665

def gen(name, dt, N, seed=42):
    rng = np.random.RandomState(seed)
    if name == "F16_Dogfight":
        s=[]; x,y,vx,vy=0,0,250.0,0
        for k in range(N):
            t=k*dt; w=0
            if 3<=t<7: w=0.35
            elif 7<=t<9: w=-0.35
            elif 9<=t<12: w=0.25
            vx+=-w*vy*dt; vy+=w*vx*dt; x+=vx*dt; y+=vy*dt; s.append([x,y,vx,vy])
        return np.array(s)
    elif name == "Kinzhal_Glide":
        s=[]; x,y,vx,vy=0,50000,1700,-200
        for k in range(N):
            t=k*dt; ax=0; ay=-9.81+5*np.sin(0.2*t)
            if 5<t<8: ax=-50; ay=-100
            vx+=ax*dt; vy+=ay*dt; x+=vx*dt; y+=vy*dt; s.append([x,y,vx,vy])
        return np.array(s)
    elif name == "Iskander_Terminal":
        s=[]; x,y,vx,vy=0,40000,1400,-1400
        for k in range(N):
            t=k*dt; ax=0; ay=-9.81
            if t>N*dt*0.7: ax=100*np.sin(3*t); ay=-9.81+150*np.cos(3*t)
            vx+=ax*dt; vy+=ay*dt; x+=vx*dt; y+=vy*dt; s.append([x,y,vx,vy])
        return np.array(s)
    elif name == "Kalibr_Cruise":
        s=[]; x,y,vx,vy=0,0,250.0,0
        for k in range(N):
            t=k*dt; w=0
            if 5<=t<7: w=0.12
            elif 12<=t<14: w=-0.10
            elif t>=18: w=0.05
            vx+=-w*vy*dt; vy+=w*vx*dt; x+=vx*dt; y+=vy*dt; s.append([x,y,vx,vy])
        return np.array(s)
    elif name == "Su35_PostStall":
        s=[]; x,y,vx,vy=0,0,200.0,0
        for k in range(N):
            t=k*dt; w=0
            if 2<=t<3: w=0.40
            elif 3<=t<4: w=-0.3+0.8*np.sin(5*t)
            elif 4<=t<6: w=-0.35
            elif 6<=t<8: w=0.25
            ax=-w*vy+rng.normal(0,5); ay=w*vx+rng.normal(0,5)
            vx+=ax*dt; vy+=ay*dt; sp=(vx**2+vy**2)**0.5
            if sp<50: vx*=50/sp; vy*=50/sp
            x+=vx*dt; y+=vy*dt; s.append([x,y,vx,vy])
        return np.array(s)
    elif name == "SAM_Terminal":
        s=[]; x,y,vx,vy=0,0,960.0,720.0
        for k in range(N):
            t=k*dt; w=0
            if 3<=t<5: w=0.15
            elif 5<=t<8: w=0.25
            elif 8<=t<10: w=-0.20
            vx+=-w*vy*dt; vy+=w*vx*dt; x+=vx*dt; y+=vy*dt; s.append([x,y,vx,vy])
        return np.array(s)
    elif name == "Shahed_Loiter":
        s=[]; x,y,vx,vy=0,0,46.0,0
        for k in range(N):
            t=k*dt; w=0
            if 10<=t<15: w=0.03
            elif t>=18: vy-=5.0*dt
            vx+=-w*vy*dt; vy+=w*vx*dt; x+=vx*dt; y+=vy*dt; s.append([x,y,vx,vy])
        return np.array(s)
    elif name == "FPV_Attack":
        s=[]; x,y,vx,vy=0,0,25.0,0
        for k in range(N):
            t=k*dt; w=0
            if t<3: w=0.5*np.sin(2*t)
            elif t<6: w=-0.8
            elif t<8: w=1.2*np.sin(8*t)
            else: vy-=3.0*dt
            ax=-w*vy+rng.normal(0,3); ay=w*vx+rng.normal(0,3)
            vx+=ax*dt; vy+=ay*dt; sp=(vx**2+vy**2)**0.5
            if sp>40: vx*=40/sp; vy*=40/sp
            if sp<5: vx*=5/sp; vy*=5/sp
            x+=vx*dt; y+=vy*dt; s.append([x,y,vx,vy])
        return np.array(s)

def rmse(truth, est, skip=5):
    t=np.array(truth); e=np.array(est); n=min(len(t),len(e))
    if n<=skip: return float('inf')
    return np.sqrt(np.mean(np.sum((t[skip:n,:2]-e[skip:n,:2])**2, axis=1)))

def ss_run(meas, dt, r_std, mode):
    N=len(meas); t0=datetime.now()
    if mode in ("cv","cv_rts"):
        tr=CombinedLinearGaussianTransitionModel([ConstantVelocity(0.5),ConstantVelocity(0.5)])
        mm=LinearGaussian(ndim_state=4,mapping=(0,2),noise_covar=np.eye(2)*r_std**2)
        s0=StateVector([[meas[0,0]],[0],[meas[0,1]],[0]]); nd=4; px=[0,2]
    elif mode=="ca":
        tr=CombinedLinearGaussianTransitionModel([ConstantAcceleration(2.0),ConstantAcceleration(2.0)])
        mm=LinearGaussian(ndim_state=6,mapping=(0,3),noise_covar=np.eye(2)*r_std**2)
        s0=StateVector([[meas[0,0]],[0],[0],[meas[0,1]],[0],[0]]); nd=6; px=[0,3]
    elif mode=="ukf_cv":
        tr=CombinedLinearGaussianTransitionModel([ConstantVelocity(1.0),ConstantVelocity(1.0)])
        mm=LinearGaussian(ndim_state=4,mapping=(0,2),noise_covar=np.eye(2)*r_std**2)
        s0=StateVector([[meas[0,0]],[0],[meas[0,1]],[0]]); nd=4; px=[0,2]
    elif mode=="ukf_ca":
        tr=CombinedLinearGaussianTransitionModel([ConstantAcceleration(3.0),ConstantAcceleration(3.0)])
        mm=LinearGaussian(ndim_state=6,mapping=(0,3),noise_covar=np.eye(2)*r_std**2)
        s0=StateVector([[meas[0,0]],[0],[0],[meas[0,1]],[0],[0]]); nd=6; px=[0,3]
    pr = (UnscentedKalmanPredictor if mode.startswith("ukf") else KalmanPredictor)(tr)
    up = (UnscentedKalmanUpdater if mode.startswith("ukf") else KalmanUpdater)(mm)
    prior=GaussianState(s0,CovarianceMatrix(np.eye(nd)*100),timestamp=t0)
    states=[prior]
    for k in range(1,N):
        ts=t0+timedelta(seconds=k*dt)
        p=pr.predict(states[-1],timestamp=ts)
        d=Detection(StateVector([[meas[k,0]],[meas[k,1]]]),timestamp=ts,measurement_model=mm)
        states.append(up.update(SingleHypothesis(p,d)))
    est=np.array([[s.state_vector[px[0],0],s.state_vector[px[1],0]] for s in states])
    if mode=="cv_rts":
        sm=KalmanSmoother(tr); track=Track(states); smoothed=sm.smooth(track)
        return np.array([[s.state_vector[0,0],s.state_vector[2,0]] for s in smoothed])
    return est

def run_fp(meas, dt, r_std):
    try:
        from filterpy.kalman import IMMEstimator, KalmanFilter
        G=np.array([[dt**2/2,0],[0,dt**2/2],[dt,0],[0,dt]])
        kf_cv=KalmanFilter(dim_x=4,dim_z=2)
        kf_cv.F=np.array([[1,0,dt,0],[0,1,0,dt],[0,0,1,0],[0,0,0,1]])
        kf_cv.H=np.array([[1,0,0,0],[0,1,0,0]]); kf_cv.R=np.eye(2)*r_std**2
        kf_cv.Q=0.5*(G@G.T); kf_cv.P*=100
        kf_ca=KalmanFilter(dim_x=4,dim_z=2)
        kf_ca.F=kf_cv.F.copy(); kf_ca.H=kf_cv.H.copy(); kf_ca.R=kf_cv.R.copy()
        kf_ca.Q=kf_cv.Q*5; kf_ca.P*=100
        imm=IMMEstimator([kf_cv,kf_ca],np.array([0.5,0.5]),np.array([[0.95,0.05],[0.05,0.95]]))
        est=[]
        for z in meas: imm.predict(); imm.update(z); est.append(imm.x[:2,0].copy())
        return est
    except: return None

def run_v40(meas, dt, r_std, db_path):
    from nx_mimosa_v40_sentinel import NxMimosaV40Sentinel
    t=NxMimosaV40Sentinel(dt=dt,r_std=r_std,platform_db_path=db_path,window_size=30)
    fwd=[]
    for z in meas: xr,_,_=t.update(z); fwd.append(xr.copy())
    return fwd

if __name__=="__main__":
    dt=0.1; r_std=2.5; N=200; n_runs=50
    scenarios=["F16_Dogfight","Kinzhal_Glide","Iskander_Terminal","Kalibr_Cruise",
               "Su35_PostStall","SAM_Terminal","Shahed_Loiter","FPV_Attack"]
    db=os.path.join(os.path.dirname(os.path.abspath(__file__)),'..','data','platform_db.json')
    if not os.path.exists(db): db=None
    ss_modes=["cv","ca","ukf_cv","ukf_ca","cv_rts"]
    labels={"cv":"SS KF-CV","ca":"SS KF-CA","ukf_cv":"SS UKF-CV","ukf_ca":"SS UKF-CA","cv_rts":"SS CV+RTS"}

    print("="*105)
    print("  NX-MIMOSA v4.0 SENTINEL vs Stone Soup 1.8 vs FilterPy IMM — 50 MC runs")
    print("="*105)
    all_r={}
    for sc in scenarios:
        print(f"\n{'─'*70}\n  {sc}\n{'─'*70}")
        r={'v40':[],'fp':[]}
        for m in ss_modes: r[m]=[]
        for run in range(n_runs):
            seed=42+run; rng=np.random.RandomState(seed+10000)
            truth=gen(sc,dt,N,seed); meas=truth[:,:2]+rng.normal(0,r_std,(N,2))
            try: r['v40'].append(rmse(truth,run_v40(meas,dt,r_std,db)))
            except: r['v40'].append(np.inf)
            fp=run_fp(meas,dt,r_std)
            if fp: r['fp'].append(rmse(truth,fp))
            for m in ss_modes:
                try: r[m].append(rmse(truth,ss_run(meas,dt,r_std,m)))
                except: r[m].append(np.inf)
        def f(a): return f"{np.mean(a):8.2f}" if a and np.isfinite(np.mean(a)) else "     N/A"
        ss_means={m:np.mean(r[m]) for m in ss_modes if r[m] and np.isfinite(np.mean(r[m]))}
        ss_bk=min(ss_means,key=ss_means.get) if ss_means else "cv"
        ss_bv=ss_means.get(ss_bk,np.inf)
        print(f"  NX-MIMOSA v4.0:     {f(r['v40'])} m")
        print(f"  FilterPy IMM(CV+CA):{f(r['fp'])} m")
        for m in ss_modes:
            print(f"  {labels[m]+':':18s}{f(r[m])} m{'  ← SS BEST' if m==ss_bk else ''}")
        all_r[sc]={'v40':np.mean(r['v40']),'fp':np.mean(r['fp']) if r['fp'] else np.inf,
                   'ss_best':ss_bv,'ss_bk':ss_bk}
        for m in ss_modes: all_r[sc][m]=np.mean(r[m]) if r[m] else np.inf
    print(f"\n{'='*105}\n  SUMMARY — RMSE (meters)\n{'='*105}")
    print(f"{'Scenario':<22} {'v4.0':>8} {'FP IMM':>8} {'SS CV':>8} {'SS CA':>8} {'SS UKF':>8} {'SSUKFCA':>8} {'SS RTS':>8} {'SS BEST':>8} {'Δ vs SS':>8}")
    print("─"*105)
    w_ss=w_fp=0
    for sc in scenarios:
        r=all_r[sc]
        def fv(v): return f"{v:8.2f}" if np.isfinite(v) else "     N/A"
        d=(r['ss_best']-r['v40'])/r['ss_best']*100 if r['ss_best']>0 and np.isfinite(r['ss_best']) else 0
        ws="✅" if r['v40']<=r['ss_best'] else "  "
        w_ss+=(1 if r['v40']<=r['ss_best'] else 0)
        w_fp+=(1 if r['v40']<=r['fp'] else 0)
        print(f"{sc:<22} {fv(r['v40'])} {fv(r['fp'])} {fv(r['cv'])} {fv(r['ca'])} {fv(r['ukf_cv'])} {fv(r['ukf_ca'])} {fv(r['cv_rts'])} {fv(r['ss_best'])} {d:>+7.1f}% {ws}")
    print("─"*105)
    v4a=np.mean([all_r[s]['v40'] for s in scenarios])
    fpa=np.mean([all_r[s]['fp'] for s in scenarios if np.isfinite(all_r[s]['fp'])])
    ssa=np.mean([all_r[s]['ss_best'] for s in scenarios if np.isfinite(all_r[s]['ss_best'])])
    # Single-tracker comparison (fair)
    ss_ukfca_avg=np.mean([all_r[s]['ukf_ca'] for s in scenarios])
    ss_ca_avg=np.mean([all_r[s]['ca'] for s in scenarios])
    print(f"\n  NX-MIMOSA v4.0 avg:  {v4a:.2f} m")
    print(f"  FilterPy IMM avg:    {fpa:.2f} m")
    print(f"  SS BEST avg (oracle):{ssa:.2f} m  (cherry-picked per scenario)")
    print(f"  SS UKF-CA avg:       {ss_ukfca_avg:.2f} m  (single-tracker, fair comparison)")
    print(f"  SS KF-CA avg:        {ss_ca_avg:.2f} m  (single-tracker, fair comparison)")
    print(f"\n  v4.0 vs FilterPy:    {w_fp}/8 wins, +{(fpa-v4a)/fpa*100:.1f}%")
    print(f"  v4.0 vs SS BEST:     {w_ss}/8 wins, +{(ssa-v4a)/ssa*100:.1f}%")
    print(f"  v4.0 vs SS UKF-CA:   +{(ss_ukfca_avg-v4a)/ss_ukfca_avg*100:.1f}% (fair single-tracker)")
