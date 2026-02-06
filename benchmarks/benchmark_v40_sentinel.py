#!/usr/bin/env python3
"""NX-MIMOSA v4.0 SENTINEL Benchmark — 8 platform-aware scenarios, 50 MC runs."""
import numpy as np, sys, os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'python'))
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
        return np.array(s),"fighter"
    elif name == "Kinzhal_Glide":
        s=[]; x,y,vx,vy=0,50000,1700,-200
        for k in range(N):
            t=k*dt; ax=0; ay=-9.81+5*np.sin(0.2*t)
            if 5<t<8: ax=-50; ay=-100
            vx+=ax*dt; vy+=ay*dt; x+=vx*dt; y+=vy*dt; s.append([x,y,vx,vy])
        return np.array(s),"hypersonic_missile"
    elif name == "Iskander_Terminal":
        s=[]; x,y,vx,vy=0,40000,1400,-1400
        for k in range(N):
            t=k*dt; ax=0; ay=-9.81
            if t>N*dt*0.7: ax=100*np.sin(3*t); ay=-9.81+150*np.cos(3*t)
            vx+=ax*dt; vy+=ay*dt; x+=vx*dt; y+=vy*dt; s.append([x,y,vx,vy])
        return np.array(s),"ballistic_missile"
    elif name == "Kalibr_Cruise":
        s=[]; x,y,vx,vy=0,0,250.0,0
        for k in range(N):
            t=k*dt; w=0
            if 5<=t<7: w=0.12
            elif 12<=t<14: w=-0.10
            elif t>=18: w=0.05
            vx+=-w*vy*dt; vy+=w*vx*dt; x+=vx*dt; y+=vy*dt; s.append([x,y,vx,vy])
        return np.array(s),"cruise_missile"
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
        return np.array(s),"fighter"
    elif name == "SAM_Terminal":
        s=[]; x,y,vx,vy=0,0,960.0,720.0
        for k in range(N):
            t=k*dt; w=0
            if 3<=t<5: w=0.15
            elif 5<=t<8: w=0.25
            elif 8<=t<10: w=-0.20
            vx+=-w*vy*dt; vy+=w*vx*dt; x+=vx*dt; y+=vy*dt; s.append([x,y,vx,vy])
        return np.array(s),"sam"
    elif name == "Shahed_Loiter":
        s=[]; x,y,vx,vy=0,0,46.0,0
        for k in range(N):
            t=k*dt; w=0
            if 10<=t<15: w=0.03
            elif t>=18: vy-=5.0*dt
            vx+=-w*vy*dt; vy+=w*vx*dt; x+=vx*dt; y+=vy*dt; s.append([x,y,vx,vy])
        return np.array(s),"loitering_munition"
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
        return np.array(s),"fpv_kamikaze"
    raise ValueError(name)

def rmse(truth, est, skip=5):
    t=np.array(truth); e=np.array(est); n=min(len(t),len(e))
    if n<=skip: return float('inf')
    return np.sqrt(np.mean(np.sum((t[skip:n,:2]-e[skip:n,:2])**2, axis=1)))

def run_fp(meas, dt, r_std):
    try:
        from filterpy.kalman import IMMEstimator, KalmanFilter
        G=np.array([[dt**2/2,0],[0,dt**2/2],[dt,0],[0,dt]])
        kf_cv = KalmanFilter(dim_x=4,dim_z=2)
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

def main():
    from nx_mimosa_v40_sentinel import NxMimosaV40Sentinel
    print("="*80); print("  NX-MIMOSA v4.0 SENTINEL Benchmark — 50 MC runs"); print("="*80)
    dt=0.1; r_std=2.5; N=200; n_runs=50
    scenarios=["F16_Dogfight","Kinzhal_Glide","Iskander_Terminal","Kalibr_Cruise",
               "Su35_PostStall","SAM_Terminal","Shahed_Loiter","FPV_Attack"]
    db=os.path.join(os.path.dirname(os.path.abspath(__file__)),'..','data','platform_db.json')
    if not os.path.exists(db): db=None
    results={}
    for sc in scenarios:
        print(f"\n{'─'*60}\n  {sc}\n{'─'*60}")
        rf,ra,rw,rfl,rfp,rcvf,rcvr,rhyb=[],[],[],[],[],[],[],[]; iok=0; plats={}; nm=[]
        for run in range(n_runs):
            seed=42+run; rng=np.random.RandomState(seed+10000)
            truth,tcat=gen(sc,dt,N,seed); meas=truth[:,:2]+rng.normal(0,r_std,(N,2))
            try:
                t=NxMimosaV40Sentinel(dt=dt,r_std=r_std,platform_db_path=db,window_size=30)
                fwd=[]
                for z in meas: xr,_,intent=t.update(z); fwd.append(xr.copy())
                win=t.get_window_smoothed_estimates(30); full=t.get_smoothed_estimates()
                adp=t.get_adaptive_best_estimates()
                cvf=t.get_cv_forward_estimates(); cvr=t.get_cv_rts_estimates()
                hyb=t.get_hybrid_best_estimates()
                rf.append(rmse(truth,fwd)); rw.append(rmse(truth,win))
                rfl.append(rmse(truth,full)); ra.append(rmse(truth,adp))
                rcvf.append(rmse(truth,cvf)); rcvr.append(rmse(truth,cvr))
                rhyb.append(rmse(truth,hyb))
                nm.append(intent.n_active_models); p=intent.platform_type
                plats[p]=plats.get(p,0)+1
                if intent.platform_category==tcat: iok+=1
            except Exception as e:
                if run==0: print(f"  ERR: {e}"); import traceback; traceback.print_exc()
                for lst in [rf,rw,rfl,ra,rcvf,rcvr,rhyb]: lst.append(np.inf)
            fp=run_fp(meas,dt,r_std)
            if fp: rfp.append(rmse(truth,fp))
        def f(a): return f"{np.mean(a):.2f}" if a and np.isfinite(np.mean(a)) else "N/A"
        means={'fwd':np.mean(rf),'adp':np.mean(ra),'win':np.mean(rw),'full':np.mean(rfl),
               'cvf':np.mean(rcvf),'cvr':np.mean(rcvr),'hyb':np.mean(rhyb)}
        best_k=min(means,key=means.get); best_v=means[best_k]
        print(f"  v4.0 IMM-Fwd:   {f(rf)} m")
        print(f"  v4.0 Adaptive:  {f(ra)} m  (NIS-gated IMM/CV realtime)")
        print(f"  v4.0 CV-Fwd:    {f(rcvf)} m  (parallel CV forward)")
        print(f"  v4.0 CV-RTS:    {f(rcvr)} m  (parallel CV+RTS offline)")
        print(f"  v4.0 Hybrid:    {f(rhyb)} m  (CV-RTS benign + IMM-fwd dynamic)")
        print(f"  v4.0 BEST:      {best_v:.2f} m ({best_k})")
        if rfp: print(f"  FilterPy IMM:   {f(rfp)} m")
        print(f"  Intent accuracy: {iok}/{n_runs} ({100*iok/n_runs:.0f}%)")
        print(f"  Platform IDs: {dict(sorted(plats.items(),key=lambda x:-x[1]))}")
        if nm: print(f"  Avg models: {np.mean(nm):.1f}")
        fpm=np.mean(rfp) if rfp else np.inf
        results[sc]={'best':best_v,'fwd':means['fwd'],'adp':means['adp'],
                     'cvf':means['cvf'],'cvr':means['cvr'],'hyb':means['hyb'],
                     'fp':fpm,'int':iok/n_runs,'stream':best_k}
    print(f"\n{'='*80}\n  SUMMARY\n{'='*80}")
    print(f"{'Scenario':<22} {'v4.0 BEST':>10} {'Stream':>6} {'v4.0 Fwd':>10} {'FilterPy':>10} {'Intent%':>8} {'Win?':>5}")
    print("─"*80)
    wins=total=0
    for sc in scenarios:
        r=results[sc]; w="✅" if r['best']<=r['fp'] else "  "; total+=1
        wins+=(1 if r['best']<=r['fp'] else 0)
        def fv(v): return f"{v:.2f}" if np.isfinite(v) else "N/A"
        print(f"{sc:<22} {fv(r['best']):>10} {r['stream']:>6} {fv(r['fwd']):>10} {fv(r['fp']):>10} {r['int']*100:>7.0f}% {w:>5}")
    print("─"*80)
    v40a=np.mean([results[s]['best'] for s in scenarios])
    fpa=np.mean([results[s]['fp'] for s in scenarios if np.isfinite(results[s]['fp'])])
    print(f"  v4.0 wins: {wins}/{total}")
    print(f"  v4.0 avg: {v40a:.2f} m | FilterPy avg: {fpa:.2f} m")
    if fpa>0: print(f"  Improvement: +{(fpa-v40a)/fpa*100:.1f}%")

if __name__=="__main__": main()
