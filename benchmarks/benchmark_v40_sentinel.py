#!/usr/bin/env python3
"""NX-MIMOSA v4.0 SENTINEL Benchmark — 8 platform-aware scenarios."""

import numpy as np
import sys, os, time
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'python'))

GRAVITY = 9.80665

def generate_scenario(name, dt, N, seed=42):
    rng = np.random.RandomState(seed)
    if name == "F16_Dogfight":
        speed=250.0; states=[]; x,y,vx,vy=0,0,speed,0
        for k in range(N):
            t=k*dt
            if t<3: omega=0
            elif t<7: omega=0.35
            elif t<9: omega=-0.35
            elif t<12: omega=0.25
            else: omega=0
            vx+=-omega*vy*dt; vy+=omega*vx*dt; x+=vx*dt; y+=vy*dt
            states.append([x,y,vx,vy])
        return np.array(states), "fighter"
    elif name == "Kinzhal_Glide":
        states=[]; x,y=0,50000; vx,vy=1700,-200
        for k in range(N):
            t=k*dt; ax=0; ay=-9.81+5*np.sin(0.2*t)
            if 5<t<8: ax=-50; ay=-100
            vx+=ax*dt; vy+=ay*dt; x+=vx*dt; y+=vy*dt
            states.append([x,y,vx,vy])
        return np.array(states), "hypersonic_missile"
    elif name == "Iskander_Terminal":
        states=[]; x,y=0,40000; vx,vy=1400,-1400
        for k in range(N):
            t=k*dt; ax=0; ay=-9.81
            if t>N*dt*0.7: ax=100*np.sin(3*t); ay=-9.81+150*np.cos(3*t)
            vx+=ax*dt; vy+=ay*dt; x+=vx*dt; y+=vy*dt
            states.append([x,y,vx,vy])
        return np.array(states), "ballistic_missile"
    elif name == "Kalibr_Cruise":
        speed=250.0; states=[]; x,y,vx,vy=0,0,speed,0
        for k in range(N):
            t=k*dt
            if t<5: omega=0
            elif t<7: omega=0.12
            elif t<12: omega=0
            elif t<14: omega=-0.10
            elif t<18: omega=0
            else: omega=0.05
            vx+=-omega*vy*dt; vy+=omega*vx*dt; x+=vx*dt; y+=vy*dt
            states.append([x,y,vx,vy])
        return np.array(states), "cruise_missile"
    elif name == "Su35_PostStall":
        speed=200.0; states=[]; x,y,vx,vy=0,0,speed,0
        for k in range(N):
            t=k*dt
            if t<2: omega=0
            elif t<3: omega=0.40
            elif t<4: omega=-0.3+0.8*np.sin(5*t)
            elif t<6: omega=-0.35
            elif t<8: omega=0.25
            else: omega=0
            vx+=(-omega*vy+rng.normal(0,5))*dt; vy+=(omega*vx+rng.normal(0,5))*dt
            sp=np.sqrt(vx**2+vy**2)
            if sp<50: vx*=50/sp; vy*=50/sp
            x+=vx*dt; y+=vy*dt; states.append([x,y,vx,vy])
        return np.array(states), "fighter"
    elif name == "SAM_Terminal":
        states=[]; x,y=0,0; vx,vy=960,720
        for k in range(N):
            t=k*dt
            if t<3: omega=0
            elif t<5: omega=0.15
            elif t<8: omega=0.25
            elif t<10: omega=-0.20
            else: omega=0
            vx+=-omega*vy*dt; vy+=omega*vx*dt; x+=vx*dt; y+=vy*dt
            states.append([x,y,vx,vy])
        return np.array(states), "sam"
    elif name == "Shahed_Loiter":
        states=[]; x,y,vx,vy=0,0,46,0
        for k in range(N):
            t=k*dt
            if t<10: omega=0
            elif t<15: omega=0.03
            elif t<18: omega=0
            else: omega=0; vy-=5*dt
            vx+=-omega*vy*dt; vy+=omega*vx*dt; x+=vx*dt; y+=vy*dt
            states.append([x,y,vx,vy])
        return np.array(states), "loitering_munition"
    elif name == "FPV_Attack":
        states=[]; x,y,vx,vy=0,0,25,0
        for k in range(N):
            t=k*dt
            if t<3: omega=0.5*np.sin(2*t)
            elif t<6: omega=-0.8
            elif t<8: omega=1.2*np.sin(8*t)
            else: omega=0; vy-=3*dt
            vx+=(-omega*vy+rng.normal(0,3))*dt; vy+=(omega*vx+rng.normal(0,3))*dt
            sp=np.sqrt(vx**2+vy**2)
            if sp>40: vx*=40/sp; vy*=40/sp
            if sp<5: vx*=5/sp; vy*=5/sp
            x+=vx*dt; y+=vy*dt; states.append([x,y,vx,vy])
        return np.array(states), "fpv_kamikaze"
    raise ValueError(name)

def compute_rmse(truth, est, skip=5):
    t=np.array(truth); e=np.array(est); n=min(len(t),len(e))
    if n<=skip: return float('inf')
    err=t[skip:n,:2]-e[skip:n,:2]; return np.sqrt(np.mean(np.sum(err**2,axis=1)))

def run_filterpy(meas, dt, r_std):
    try:
        from filterpy.kalman import IMMEstimator, KalmanFilter
        kf1=KalmanFilter(4,2); kf1.F=np.array([[1,0,dt,0],[0,1,0,dt],[0,0,1,0],[0,0,0,1]])
        kf1.H=np.array([[1,0,0,0],[0,1,0,0]]); kf1.R=np.eye(2)*r_std**2
        G=np.array([[dt**2/2,0],[0,dt**2/2],[dt,0],[0,dt]]); kf1.Q=0.5*(G@G.T); kf1.P*=100
        kf2=KalmanFilter(4,2); kf2.F=kf1.F.copy(); kf2.H=kf1.H.copy()
        kf2.R=kf1.R.copy(); kf2.Q=kf1.Q*5; kf2.P*=100
        imm=IMMEstimator([kf1,kf2],np.array([.5,.5]),np.array([[.95,.05],[.05,.95]]))
        est=[]
        for z in meas: imm.predict(); imm.update(z); est.append(imm.x[:2,0].copy())
        return est
    except: return None

def main():
    from nx_mimosa_v40_sentinel import NxMimosaV40Sentinel
    print("="*80)
    print("  NX-MIMOSA v4.0 SENTINEL Benchmark — 50 MC runs")
    print("="*80)
    dt=0.1; r_std=2.5; N=200; n_runs=50
    scenarios=["F16_Dogfight","Kinzhal_Glide","Iskander_Terminal","Kalibr_Cruise",
               "Su35_PostStall","SAM_Terminal","Shahed_Loiter","FPV_Attack"]
    db_path=os.path.join(os.path.dirname(os.path.abspath(__file__)),'..','data','platform_db.json')
    if not os.path.exists(db_path): db_path=None

    results={}
    for sc in scenarios:
        print(f"\n{'─'*60}\n  {sc}\n{'─'*60}")
        rf,rw,rfl,rfp=[],[],[],[]
        ic=0; pids={}; anm=[]
        for run in range(n_runs):
            seed=42+run; truth,true_cat=generate_scenario(sc,dt,N,seed)
            rng=np.random.RandomState(seed+10000)
            meas=truth[:,:2]+rng.normal(0,r_std,(N,2))
            try:
                tracker=NxMimosaV40Sentinel(dt=dt,r_std=r_std,platform_db_path=db_path,
                                             window_size=30,prune_threshold=0.03)
                fwd_e=[]; il=[]
                for z in meas:
                    xr,Pr,intent=tracker.update(z)
                    fwd_e.append(xr.copy())
                    il.append(intent)
                win_e=tracker.get_window_smoothed_estimates(30)
                full_e=tracker.get_smoothed_estimates()
                rf.append(compute_rmse(truth,fwd_e))
                rw.append(compute_rmse(truth,win_e))
                rfl.append(compute_rmse(truth,full_e))
                last=il[-1]
                anm.append(last.n_active_models)
                p=last.platform_type; pids[p]=pids.get(p,0)+1
                if last.platform_category==true_cat: ic+=1
            except Exception as e:
                if run==0: print(f"  ERR: {e}")
                rf.append(float('inf')); rw.append(float('inf')); rfl.append(float('inf'))
            fp=run_filterpy(meas,dt,r_std)
            if fp: rfp.append(compute_rmse(truth,fp))

        def f(a): return f"{np.mean(a):.2f}" if a and np.isfinite(np.mean(a)) else "N/A"
        # Pick best stream
        fwd_m=np.mean(rf) if rf else float('inf')
        win_m=np.mean(rw) if rw else float('inf')
        full_m=np.mean(rfl) if rfl else float('inf')
        best_m=min(fwd_m,win_m,full_m)
        best_lbl="fwd" if best_m==fwd_m else ("win30" if best_m==win_m else "full")

        print(f"  v4.0 Forward:   {f(rf)} m")
        print(f"  v4.0 Window-30: {f(rw)} m")
        print(f"  v4.0 Full:      {f(rfl)} m")
        print(f"  v4.0 BEST:      {f([best_m])} m ({best_lbl})")
        if rfp: print(f"  FilterPy IMM:   {f(rfp)} m")
        print(f"  Intent accuracy: {ic}/{n_runs} ({100*ic/n_runs:.0f}%)")
        print(f"  Platform IDs: {dict(sorted(pids.items(),key=lambda x:-x[1]))}")
        if anm: print(f"  Avg models: {np.mean(anm):.1f}")

        results[sc]={'fwd':fwd_m,'win':win_m,'full':full_m,'best':best_m,
                      'fp':np.mean(rfp) if rfp else float('inf'),'intent':ic/n_runs}

    print("\n"+"="*80)
    print("  SUMMARY")
    print("="*80)
    print(f"{'Scenario':<22} {'v4.0 BEST':>10} {'v4.0 Fwd':>10} {'v4.0 W30':>10} {'FilterPy':>10} {'Intent%':>8} {'Win?':>5}")
    print("─"*80)
    wins=0; total=0
    for sc in scenarios:
        r=results[sc]; fp=r['fp']
        w="✅" if np.isfinite(r['best']) and np.isfinite(fp) and r['best']<fp else "  "
        if np.isfinite(r['best']) and np.isfinite(fp):
            total+=1
            if r['best']<=fp: wins+=1
        def fv(v): return f"{v:.2f}" if np.isfinite(v) else "N/A"
        print(f"{sc:<22} {fv(r['best']):>10} {fv(r['fwd']):>10} {fv(r['win']):>10} {fv(r['fp']):>10} {r['intent']*100:>7.0f}% {w}")
    print("─"*80)
    print(f"  v4.0 wins: {wins}/{total}")

    if total>0:
        v4a=np.mean([results[s]['best'] for s in scenarios if np.isfinite(results[s]['best'])])
        fpa=np.mean([results[s]['fp'] for s in scenarios if np.isfinite(results[s]['fp'])])
        print(f"  v4.0 avg: {v4a:.2f} m | FilterPy avg: {fpa:.2f} m")
        if fpa>0: print(f"  Improvement: {(fpa-v4a)/fpa*100:+.1f}%")

if __name__=="__main__": main()
