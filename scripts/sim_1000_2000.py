#!/usr/bin/env python3
"""
NX-MIMOSA v6.0.1 — Realistic Air Surveillance Simulation
1,000 and 2,000 aircraft over Central European airspace.
Generates docs/SIMULATION_v601_1000_2000.md results.

Usage: python scripts/sim_1000_2000.py
"""
import sys, time, json, numpy as np
sys.path.insert(0, 'python')
sys.path.insert(0, 'cpp/build')

DT, R_STD, SEED = 5.0, 150.0, 42

def generate_aircraft(N, rng):
    aircraft, cats = [], {'commercial':0,'ga':0,'military':0,'helicopter':0}
    for _ in range(N):
        r = rng.random()
        if r < 0.85: cat,alt,spd,tr = 'commercial',rng.uniform(8000,13000),rng.uniform(200,270),rng.uniform(0,0.5)
        elif r < 0.95: cat,alt,spd,tr = 'ga',rng.uniform(1500,5000),rng.uniform(50,100),rng.uniform(0,2.0)
        elif r < 0.98: cat,alt,spd,tr = 'military',rng.uniform(3000,15000),rng.uniform(200,500),rng.uniform(0,3.0)
        else: cat,alt,spd,tr = 'helicopter',rng.uniform(100,2000),rng.uniform(30,80),rng.uniform(0,5.0)
        cats[cat] += 1
        h = rng.uniform(0, 2*np.pi)
        aircraft.append({'pos': np.array([rng.uniform(-4e5,4e5), rng.uniform(-4e5,4e5), alt]),
                        'vel': np.array([spd*np.cos(h), spd*np.sin(h), rng.normal(0,2)]),
                        'turn_rate': tr*np.pi/180*rng.choice([-1,1]), 'cat': cat})
    return aircraft, cats

def propagate(ac, dt):
    w = ac['turn_rate']; c, s = np.cos(w*dt), np.sin(w*dt)
    vx, vy = ac['vel'][0], ac['vel'][1]
    nvx, nvy = (c*vx-s*vy, s*vx+c*vy) if abs(w)>1e-6 else (vx, vy)
    ac['pos'][:2] += np.array([(vx+nvx)/2*dt, (vy+nvy)/2*dt])
    ac['pos'][2] = np.clip(ac['pos'][2] + ac['vel'][2]*dt, 50, 20000)
    ac['vel'][0], ac['vel'][1] = nvx, nvy

def run(N, n_scans=20):
    rng = np.random.default_rng(SEED)
    ac, cats = generate_aircraft(N, rng)
    scans, gt = [], []
    for s in range(n_scans):
        m = []
        for a in ac:
            if s > 0: propagate(a, DT)
            noise = rng.normal(0, R_STD, 3); noise[2] *= 0.5
            m.append(a['pos'] + noise)
        for _ in range(5): m.append(rng.uniform(-4e5, 4e5, 3))
        scans.append(np.array(m))
        gt.append(np.array([a['pos'].copy() for a in ac]))

    try:
        import _nx_core as nx
        cfg = nx.TrackerConfig(); cfg.dt = DT; cfg.r_std = R_STD
        t = nx.MultiTargetTracker(cfg)
        ct = [t.process_scan(m, s*DT) for s, m in enumerate(scans)]
        print(f"  C++ {N}: {np.mean(ct[3:]):.1f} ms mean, {t.n_confirmed} confirmed")
    except ImportError:
        print("  C++ not available, run: cd cpp && pip install .")

    from nx_mimosa_mtt import MultiTargetTracker as PyTracker
    py = PyTracker(dt=DT, r_std=R_STD, domain='air')
    pt = []
    for m in scans:
        t0 = time.perf_counter(); py.process_scan(m); pt.append((time.perf_counter()-t0)*1000)
    print(f"  Py  {N}: {np.mean(pt[3:]):.1f} ms mean, {len(py.confirmed_tracks)} confirmed")

if __name__ == '__main__':
    print("NX-MIMOSA v6.0.1 — Air Surveillance Simulation (seed 42)")
    run(1000)
    run(2000)
