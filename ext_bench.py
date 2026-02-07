import numpy as np, sys, os, time, json, requests
from collections import defaultdict
sys.path.insert(0, './python')
os.chdir('/home/claude/nx-mimosa')
from nx_mimosa_mtt import (
    KalmanFilter3D, IMM3D, MultiTargetTracker, TrackManagerConfig,
    make_cv3d_matrices, compute_gospa, compute_ospa,
)

R_EARTH = 6371000.0; SEED = 42; NOISE = 150.0
np.random.seed(SEED)

MIL_ICAO = {
    "US_DoD":[(0xAE0000,0xAFFFFF)], "UK_RAF":[(0x43C000,0x43CFFF)],
    "FR_MIL":[(0x3A8000,0x3AFFFF)], "DE_MIL":[(0x3F4000,0x3F7FFF)],
    "NATO":[(0x478000,0x47FFFF)], "PL_MIL":[(0x48D000,0x48DFFF)],
    "IT_MIL":[(0x33F000,0x33FFFF)], "ES_MIL":[(0x340000,0x340FFF)],
}
MIL_PFX = ['RCH','RRR','REACH','SPAR','SAM','DUKE','HAWK','VIPER','NAVY','PAT',
           'RAF','ASCOT','CTM','FAF','GAF','IAM','PLF','NSZ','BAF','NAF','TURK','HAF']

def lla2enu(lat,lon,alt,lat0=49,lon0=17.5):
    return np.array([np.radians(lon-lon0)*R_EARTH*np.cos(np.radians(lat0)),
                     np.radians(lat-lat0)*R_EARTH, alt])

def is_mil(icao_hex, cs):
    try: iv = int(icao_hex,16)
    except: return False,None
    for f,rngs in MIL_ICAO.items():
        for lo,hi in rngs:
            if lo<=iv<=hi: return True,f
    if cs:
        for p in MIL_PFX:
            if cs.strip().upper().startswith(p): return True,p
    return False,None

def fetch():
    r = requests.get("https://opensky-network.org/api/states/all?lamin=43&lamax=55&lomin=5&lomax=30", timeout=30)
    r.raise_for_status()
    d = r.json()
    return d.get('states',[]), d.get('time',0)

# Collect 20 snapshots @ 10s
N=20; DT=10.0
print(f"Collecting {N} snapshots @ {DT}s...")
snaps = []
for i in range(N):
    t0=time.time()
    try:
        s,ts = fetch()
        snaps.append(s)
        print(f"  {i+1:2d}/{N}: {len(s):4d} aircraft ({time.time()-t0:.1f}s)")
    except Exception as e:
        snaps.append([])
        print(f"  {i+1:2d}/{N}: FAILED {e}")
    if i < N-1:
        w = max(0, DT-(time.time()-t0))
        if w>0: time.sleep(w)

# Parse
all_icaos = set(); mil = {}; max_ac = 0
scan_data = []
for si,states in enumerate(snaps):
    meas=[]; ics=[]
    for s in (states or []):
        ic,cs,lon,lat,ba,ga,vel = s[0],s[1],s[5],s[6],s[7],s[13],s[9]
        if lat is None or lon is None: continue
        alt = ba or ga or 10000
        enu = lla2enu(lat,lon,alt) + np.random.randn(3)*NOISE
        meas.append(enu); ics.append(ic); all_icaos.add(ic)
        m,f = is_mil(ic, cs or "")
        if m and ic not in mil:
            mil[ic] = {'cs':(cs or '').strip(),'force':f,'alt':alt,'vel':vel or 0,'scans':[]}
        if ic in mil: mil[ic]['scans'].append(si); mil[ic]['alt']=alt
    if len(meas)>max_ac: max_ac=len(meas)
    scan_data.append(np.array(meas) if meas else np.zeros((0,3)))

print(f"\nUnique ICAO: {len(all_icaos)} | Peak: {max_ac} | Military: {len(mil)}")

# Track
cfg = TrackManagerConfig()
cfg.confirm_m=2; cfg.confirm_n=3; cfg.coast_max=3; cfg.gate_sigma=5.0
tracker = MultiTargetTracker(dt=DT, r_std=NOISE, domain="atc", association="gnn", config=cfg)

times=[]; ntgts=[]; ntrks=[]; drs=[]; td=defaultdict(int)
print(f"\nTracking...")
for si,m in enumerate(scan_data):
    if len(m)==0: continue
    t0=time.perf_counter()
    trks = tracker.process_scan(m)
    ms=(time.perf_counter()-t0)*1000
    conf = [t for t in trks if t.confirmed]
    dr = len(conf)/len(m) if len(m)>0 else 0
    times.append(ms); ntgts.append(len(m)); ntrks.append(len(conf)); drs.append(dr)
    for t in conf: td[t.track_id]+=1
    if si%4==0 or si==len(scan_data)-1:
        print(f"  Scan {si+1:2d}: {len(m):4d} meas → {len(conf):4d} tracks ({dr:.1%}) | {ms:.0f}ms")

T=np.array(times); NT=np.array(ntgts); DR=np.array(drs)
dur=list(td.values())

print(f"\n{'='*60}")
print(f"  EXTENDED BENCHMARK RESULTS")
print(f"{'='*60}")
print(f"  Scans processed:        {len(times)}")
print(f"  Total duration:         {len(times)*DT/60:.1f} min")
print(f"  Peak targets:           {NT.max()}")
print(f"  Mean targets/scan:      {NT.mean():.0f}")
print(f"  Unique ICAO24:          {len(all_icaos)}")
print(f"  Military identified:    {len(mil)}")
print(f"  Mean scan time:         {T.mean():.0f} ms")
print(f"  Median scan time:       {np.median(T):.0f} ms")
print(f"  P95 scan time:          {np.percentile(T,95):.0f} ms")
print(f"  Max scan time:          {T.max():.0f} ms")
print(f"  Mean detection rate:    {DR.mean():.1%}")
print(f"  Min detection rate:     {DR.min():.1%}")
print(f"  Total tracks created:   {len(td)}")
print(f"  Long tracks (5+ scans): {sum(1 for d in dur if d>=5)}")
print(f"  Scan time drift:        {((T[-3:].mean()-T[:3].mean())/T[:3].mean()*100):+.1f}%")

if mil:
    print(f"\n  MILITARY DETAIL:")
    print(f"  {'ICAO':<10} {'Callsign':<12} {'Force':<10} {'FL':<8} {'Scans':<8}")
    for ic,info in sorted(mil.items(), key=lambda x:-len(x[1]['scans'])):
        fl = f"FL{int(info['alt']*3.28084/100)}"
        print(f"  {ic:<10} {info['cs']:<12} {info['force']:<10} {fl:<8} {len(info['scans'])}/{len(times)}")

# Timeline
print(f"\n  SCAN TIMELINE:")
for i in range(len(times)):
    bar = '█' * int(times[i]/30)
    print(f"  {i+1:2d} | {ntgts[i]:4d}→{ntrks[i]:4d} | {drs[i]:.0%} | {times[i]:6.0f}ms |{bar}")

res = {
    "n_scans":len(times),"peak":int(NT.max()),"mean_targets":float(NT.mean()),
    "unique_icao":len(all_icaos),"military":len(mil),
    "mean_ms":float(T.mean()),"p50_ms":float(np.median(T)),"p95_ms":float(np.percentile(T,95)),
    "max_ms":float(T.max()),"mean_dr":float(DR.mean()),"min_dr":float(DR.min()),
    "long_tracks":sum(1 for d in dur if d>=5), "total_tracks":len(td),
    "mil_detail":{ic:{'cs':i['cs'],'force':i['force'],'scans':len(i['scans'])} for ic,i in mil.items()}
}
with open('docs/BENCHMARK_v591_extended.json','w') as f: json.dump(res,f,indent=2)
print(f"\nSaved: docs/BENCHMARK_v591_extended.json")
