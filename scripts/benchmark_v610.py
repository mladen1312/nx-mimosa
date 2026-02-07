#!/usr/bin/env python3
"""
NX-MIMOSA v6.1.0 — Full-Scale Benchmark (C++ Core + ECCM)
===========================================================
1,000 / 2,000 / 5,000 aircraft. C++ _nx_core for ALL tracking.
Python ECCM overlay for environment classification analysis only.

Author: Dr. Mladen Mešter · Nexellum d.o.o. · 2026
"""
import sys, os, time, json
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'python'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'cpp', 'build'))

import _nx_core as nx
from nx_mimosa_eccm import ECCMPipeline, EnvironmentClass

DT        = 5.0
R_STD     = 150.0
Q_BASE    = 1.0
SEED      = 42
N_SCANS   = 20
N_CLUTTER = 10

JAM_START     = 8
JAM_FRAC      = 0.05
JAM_NOISE_X   = 8.0
RGPO_RATE     = 300.0


def generate_aircraft(N, rng):
    ac, cats = [], {'commercial': 0, 'ga': 0, 'military': 0, 'helicopter': 0}
    area = 4e5 if N <= 2000 else 6e5
    for _ in range(N):
        r = rng.random()
        if   r < 0.82: cat,alt,spd,tr = 'commercial',rng.uniform(8e3,13e3),rng.uniform(200,270),rng.uniform(0,0.3)
        elif r < 0.93: cat,alt,spd,tr = 'ga',rng.uniform(1500,5e3),rng.uniform(50,100),rng.uniform(0,1.5)
        elif r < 0.97: cat,alt,spd,tr = 'military',rng.uniform(3e3,15e3),rng.uniform(200,500),rng.uniform(0,3.0)
        else:          cat,alt,spd,tr = 'helicopter',rng.uniform(100,2e3),rng.uniform(30,80),rng.uniform(0,5.0)
        cats[cat] += 1
        h = rng.uniform(0, 2*np.pi)
        ac.append({
            'pos': np.array([rng.uniform(-area,area), rng.uniform(-area,area), alt]),
            'vel': np.array([spd*np.cos(h), spd*np.sin(h), rng.normal(0,1.5)]),
            'w': tr*np.pi/180*rng.choice([-1,1]),
            'cat': cat, 'jammed': False, 'jam_type': None,
        })
    return ac, cats


def propagate(ac, dt):
    w = ac['w']
    c, s = np.cos(w*dt), np.sin(w*dt)
    vx, vy = ac['vel'][0], ac['vel'][1]
    nvx = c*vx - s*vy if abs(w) > 1e-6 else vx
    nvy = s*vx + c*vy if abs(w) > 1e-6 else vy
    ac['pos'][:2] += np.array([(vx+nvx)/2*dt, (vy+nvy)/2*dt])
    ac['pos'][2] = np.clip(ac['pos'][2] + ac['vel'][2]*dt, 50, 20000)
    ac['vel'][0], ac['vel'][1] = nvx, nvy


def gospa(trk_pos, gt_pos, c=10000.0, p=2):
    nt, ne = len(gt_pos), len(trk_pos)
    if nt == 0 and ne == 0:
        return dict(gospa=0, loc=0, missed=0, false=0, n_asgn=0, n_miss=0, n_false=0)
    if ne == 0:
        return dict(gospa=(nt*c**p)**(1/p), loc=0, missed=(nt*c**p)**(1/p), false=0,
                    n_asgn=0, n_miss=nt, n_false=0)
    if nt == 0:
        return dict(gospa=(ne*c**p)**(1/p), loc=0, missed=0, false=(ne*c**p)**(1/p),
                    n_asgn=0, n_miss=0, n_false=ne)
    ta, ga = np.array(trk_pos), np.array(gt_pos)
    used = set()
    assignments = []
    for ti in range(nt):
        d = np.linalg.norm(ta - ga[ti], axis=1)
        for ej in np.argsort(d):
            if ej not in used and d[ej] < c:
                assignments.append(d[ej]); used.add(ej); break
    na = len(assignments)
    nm, nf = nt - na, ne - na
    lc = sum(d**p for d in assignments)
    total = lc + nm*c**p + nf*c**p
    return dict(gospa=total**(1/p), loc=lc**(1/p) if lc > 0 else 0,
                missed=(nm*c**p)**(1/p) if nm else 0, false=(nf*c**p)**(1/p) if nf else 0,
                n_asgn=na, n_miss=nm, n_false=nf)


# ═══════════════════════════════════════════════════════════════
def run(N, inject_ecm=True):
    rng = np.random.default_rng(SEED)
    ac, cats = generate_aircraft(N, rng)

    n_jam = max(1, int(N * JAM_FRAC))
    ji = rng.choice(N, n_jam, replace=False)
    jt = rng.choice(['NOISE','RGPO','DRFM'], n_jam)
    for i, idx in enumerate(ji):
        ac[idx]['jammed'] = True
        ac[idx]['jam_type'] = jt[i]

    print(f"\n{'='*70}")
    print(f"  NX-MIMOSA v6.1.0  C++ CORE  —  {N:,} AIRCRAFT")
    print(f"{'='*70}")
    print(f"  Mix: {cats}  |  Jammed: {n_jam}  |  Scans: {N_SCANS}")

    # ── C++ tracker ──
    cfg = nx.TrackerConfig()
    cfg.dt = DT; cfg.r_std = R_STD; cfg.q_base = Q_BASE
    cfg.gate_threshold  = 25.0 if N <= 2000 else 20.0
    cfg.coarse_gate_m   = 25000.0 if N <= 2000 else 30000.0
    cfg.min_separation  = 800.0 if N <= 2000 else 1200.0
    cfg.confirm_hits    = 3
    cfg.delete_misses   = 6
    cfg.coast_max       = 4
    tracker = nx.MultiTargetTracker(cfg)

    eccm = ECCMPipeline(ml_window=10) if inject_ecm else None

    scan_ms, gscores, rms_list, n_conf_list = [], [], [], []
    eccm_ok, eccm_tot = 0, 0

    for s in range(N_SCANS):
        if s > 0:
            for a in ac: propagate(a, DT)

        area = 4e5 if N <= 2000 else 6e5
        meas, gt = [], []
        for ai, a in enumerate(ac):
            noise = rng.normal(0, R_STD, 3); noise[2] *= 0.5
            mp = a['pos'].copy() + noise
            if a['jammed'] and s >= JAM_START:
                if a['jam_type'] == 'NOISE':
                    mp = a['pos'].copy() + rng.normal(0, R_STD*JAM_NOISE_X, 3)
                elif a['jam_type'] == 'RGPO':
                    d = a['pos'] / (np.linalg.norm(a['pos'])+1e-6)
                    mp = a['pos'].copy() + noise + d * RGPO_RATE * (s - JAM_START)
                elif a['jam_type'] == 'DRFM':
                    meas.append(a['pos'].copy() + rng.normal(0, 2000, 3))
            meas.append(mp); gt.append(a['pos'].copy())

        for _ in range(N_CLUTTER):
            meas.append(rng.uniform(-area, area, 3))

        marr = np.array(meas)

        # ── C++ process_scan ── timing comes from C++ chrono inside binding
        t0 = time.perf_counter()
        cpp_ms = tracker.process_scan(marr, s * DT)
        wall_ms = (time.perf_counter() - t0) * 1000
        scan_ms.append(wall_ms)

        ids, states = tracker.get_confirmed_states()
        nc = len(ids)
        n_conf_list.append(nc)

        tp = [states[i, :3] for i in range(nc)]
        g = gospa(tp, gt)
        gscores.append(g)

        # RMS
        if g['n_asgn'] > 0:
            ga2, ta2 = np.array(gt), np.array(tp)
            errs = []
            used2 = set()
            for ti2 in range(len(gt)):
                d2 = np.linalg.norm(ta2 - ga2[ti2], axis=1)
                for ej2 in np.argsort(d2):
                    if ej2 not in used2 and d2[ej2] < 10000:
                        errs.append(d2[ej2]); used2.add(ej2); break
            if errs:
                rms_list.append(float(np.sqrt(np.mean(np.array(errs)**2))))

        # ── ECCM overlay (lightweight Python — not timed as tracking) ──
        if eccm and s >= 3 and nc > 0:
            for ai, a in enumerate(ac):
                if ai >= nc: break
                mi = min(ai, len(meas)-1)
                err = np.linalg.norm(np.array(meas[mi]) - tp[ai])
                nis = (err / R_STD)**2
                innov = np.array(meas[mi]) - tp[ai]
                vel = states[ai, 3:6] if ai < len(states) else np.zeros(3)
                res = eccm.update(track_id=ai, nis=nis, innovation=innov, velocity=vel, was_hit=True)
                if a['jammed'] and s >= JAM_START:
                    eccm_tot += 1
                    if res['env_class'] != EnvironmentClass.CLEAR:
                        eccm_ok += 1

        if s % 4 == 0 or s == N_SCANS - 1:
            rms_s = f"{rms_list[-1]:.0f}" if rms_list else "—"
            print(f"  Scan {s+1:2d}/{N_SCANS}: {len(marr):5d} meas → {nc:5d} trk "
                  f"| {wall_ms:6.1f} ms | GOSPA={g['gospa']:7.0f} | RMS={rms_s} m")

    sk = 3
    vt = scan_ms[sk:]
    vg = gscores[sk:]
    vr = rms_list[sk:] if len(rms_list) > sk else rms_list
    det = np.mean([g['n_asgn']/N for g in vg])
    epdp = eccm_ok / max(eccm_tot, 1)

    res = dict(
        N=N, cats=cats, n_jam=n_jam,
        mean_ms=float(np.mean(vt)), p95_ms=float(np.percentile(vt,95)),
        max_ms=float(np.max(vt)), min_ms=float(np.min(vt)),
        total_s=float(sum(scan_ms)/1000), us_per_tgt=float(np.mean(vt)/N*1000),
        mean_gospa=float(np.mean([g['gospa'] for g in vg])),
        mean_rms=float(np.mean(vr)) if vr else 0,
        det_rate=float(det), final_conf=int(n_conf_list[-1]),
        created=tracker.total_created,
        mean_asgn=float(np.mean([g['n_asgn'] for g in vg])),
        mean_miss=float(np.mean([g['n_miss'] for g in vg])),
        mean_false=float(np.mean([g['n_false'] for g in vg])),
        eccm_pd=epdp, eccm_ok=eccm_ok, eccm_tot=eccm_tot,
        scan_ms=scan_ms, n_conf=n_conf_list,
    )

    print(f"\n  {'─'*50}")
    print(f"  {N:,} AC  RESULTS (C++ core):")
    print(f"    Mean:  {res['mean_ms']:.1f} ms  |  P95: {res['p95_ms']:.1f} ms  |  Max: {res['max_ms']:.1f} ms")
    print(f"    µs/target: {res['us_per_tgt']:.1f}")
    print(f"    GOSPA: {res['mean_gospa']:.0f} m  |  RMS: {res['mean_rms']:.0f} m")
    print(f"    Detection: {res['det_rate']*100:.1f}%  |  Confirmed: {res['final_conf']}/{N}")
    if inject_ecm:
        print(f"    ECCM Pd: {res['eccm_pd']*100:.1f}% ({eccm_ok}/{eccm_tot})")
    return res


# ═══════════════════════════════════════════════════════════════
def report(rl):
    r1 = rl[0] if len(rl) > 0 else None
    r2 = rl[1] if len(rl) > 1 else None
    r5 = rl[2] if len(rl) > 2 else None

    def v(r, k, f=".1f"):
        if r is None: return "—"
        val = r[k]
        if f == "d": return f"{int(val):,}"
        if f == "%": return f"{val*100:.1f}%"
        return f"{val:{f}}"

    L = [
        "# NX-MIMOSA v6.1.0 — Full-Scale C++ Benchmark",
        "",
        f"**Date:** {time.strftime('%Y-%m-%d %H:%M UTC', time.gmtime())}  ",
        f"**Engine:** C++ `_nx_core` (Eigen3 + OpenMP + KDTree + Bertsekas Auction)  ",
        f"**IMM:** 4-model (CV/CA/CT+/CT−) per track  ",
        f"**ECCM:** Python ML-CFAR overlay (6-feature classifier, not in timing loop)  ",
        f"**Platform:** {os.popen('uname -m').read().strip()}, {os.cpu_count()} cores  ",
        "",
        "## Summary",
        "",
        "| Metric | 1,000 AC | 2,000 AC | 5,000 AC |",
        "|--------|----------|----------|----------|",
        f"| Mean scan (ms) | {v(r1,'mean_ms')} | {v(r2,'mean_ms')} | {v(r5,'mean_ms')} |",
        f"| P95 scan (ms) | {v(r1,'p95_ms')} | {v(r2,'p95_ms')} | {v(r5,'p95_ms')} |",
        f"| Max scan (ms) | {v(r1,'max_ms')} | {v(r2,'max_ms')} | {v(r5,'max_ms')} |",
        f"| µs / target | {v(r1,'us_per_tgt')} | {v(r2,'us_per_tgt')} | {v(r5,'us_per_tgt')} |",
        f"| GOSPA (m) | {v(r1,'mean_gospa','.0f')} | {v(r2,'mean_gospa','.0f')} | {v(r5,'mean_gospa','.0f')} |",
        f"| RMS error (m) | {v(r1,'mean_rms','.0f')} | {v(r2,'mean_rms','.0f')} | {v(r5,'mean_rms','.0f')} |",
        f"| Detection rate | {v(r1,'det_rate','%')} | {v(r2,'det_rate','%')} | {v(r5,'det_rate','%')} |",
        f"| Final confirmed | {v(r1,'final_conf','d')} | {v(r2,'final_conf','d')} | {v(r5,'final_conf','d')} |",
        f"| ECCM Pd | {v(r1,'eccm_pd','%')} | {v(r2,'eccm_pd','%')} | {v(r5,'eccm_pd','%')} |",
        "",
    ]

    # Capability delta
    L += [
        "## v6.0.1 → v6.1.0 Capability Delta",
        "",
        "| Capability | v6.0.1 | v6.1.0 | QEDMMA Source |",
        "|-----------|--------|--------|---------------|",
        "| ECM classification | 4 binary detectors | 6-feature ML + 4 environment classes | ml_cfar_engine.sv |",
        "| Features extracted | NIS, range-rate, jitter | power ratio, guard ratio, kurtosis, lag-1 corr, range/Doppler deriv | ml_cfar_engine.sv |",
        "| R-matrix adaptation | None | Auto 1×–10× by environment | integration_controller.sv |",
        "| Gate adaptation | Fixed 4σ | Dynamic 3σ–8σ | integration_controller.sv |",
        "| Coast adaptation | Fixed | Dynamic 8–20 scans | integration_controller.sv |",
        "| Bearing-only mode | No | Auto in heavy jamming | integration_controller.sv |",
        "| Jammer localization | No | TDOA (Chan-Ho + Gauss-Newton + KF) | jammer_localizer.sv |",
        "| Integration levels | 1 | 4 (NORMAL/ELEVATED/HIGH/MAXIMUM) | integration_controller.sv |",
        "| Classes | 71 | 78 | — |",
        "| LOC | 11,493 | 12,980 | — |",
        "| Tests | 352 | 397 | — |",
        "",
    ]

    # Scaling
    if r1 and r2 and r5:
        t1, t2, t5 = r1['mean_ms'], r2['mean_ms'], r5['mean_ms']
        a15 = np.log(t5/t1) / np.log(5000/1000) if t1 > 0 else 0
        a12 = np.log(t2/t1) / np.log(2000/1000) if t1 > 0 else 0
        a25 = np.log(t5/t2) / np.log(5000/2000) if t2 > 0 else 0
        L += [
            "## Scaling Analysis",
            "",
            f"T ∝ N^α:  1K→2K α={a12:.2f}  |  2K→5K α={a25:.2f}  |  1K→5K α={a15:.2f}",
            "",
            f"Average α = {a15:.2f} → {'sub-quadratic (KDTree + sparse auction effective)' if a15 < 1.8 else 'super-linear'}.",
            f"Projected 10K targets: ~{t5*(10000/5000)**a15:.0f} ms/scan.",
            "",
        ]

    # Per-scenario
    for r in rl:
        if r is None: continue
        L += [
            f"## {r['N']:,} Aircraft Detail",
            "",
            f"Mix: {r['cats']}  |  Jammed: {r['n_jam']}",
            "",
            "```",
        ]
        for i, t in enumerate(r['scan_ms']):
            bar = '█' * int(t / max(r['scan_ms']) * 40)
            L.append(f"  Scan {i+1:2d}: {t:7.1f} ms  {bar}  {r['n_conf'][i]:5d} trk")
        L += ["```", ""]

    L += [
        "## Methodology",
        "",
        "- **Tracking engine**: C++ `_nx_core` — Eigen3, OpenMP, KDTree O(N log N) gating,",
        "  Bertsekas sparse auction O(N·k) assignment, 4-model IMM per track",
        "- **ECCM analysis**: Python overlay (NOT included in timing) — ML-CFAR 6-feature",
        "  environment classification + adaptive parameter recommendation",
        "- **ECM injection**: 5% targets jammed from scan 8 (NOISE 8×, RGPO 300m/s, DRFM ghost)",
        f"- **GOSPA**: c=10 km, p=2, greedy assignment | **Seed**: {SEED}",
        "",
        "---",
        "*NX-MIMOSA v6.1.0 · Copyright © 2026 Nexellum d.o.o.*",
    ]
    return '\n'.join(L)


if __name__ == '__main__':
    print("=" * 70)
    print("  NX-MIMOSA v6.1.0 — C++ CORE BENCHMARK")
    print("=" * 70)

    results = []
    for N in [1000, 2000, 5000]:
        results.append(run(N))

    md = report(results)
    os.makedirs('docs', exist_ok=True)
    with open('docs/BENCHMARK_v610_FULL.md', 'w') as f:
        f.write(md)

    jr = [{k: v for k, v in r.items() if k not in ('scan_ms','n_conf')} for r in results]
    with open('docs/BENCHMARK_v610_FULL.json', 'w') as f:
        json.dump(jr, f, indent=2, default=str)

    print(f"\n  Report: docs/BENCHMARK_v610_FULL.md")
    print(f"  JSON:   docs/BENCHMARK_v610_FULL.json")
