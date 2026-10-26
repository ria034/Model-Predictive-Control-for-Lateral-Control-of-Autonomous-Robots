import copy, json, time, numpy as np, sys
from multiprocessing import Pool
from sim import *
CASES = {
  'nominal':  {},
  'noise':    dict(noise=True),
  'mismatch': dict(mismatch=True, mu_scale=0.85),
  'delay':    dict(delay_steps=2),
  'delay_nocomp': dict(delay_steps=2, compensate_delay=False),
  'combined': dict(noise=True, mismatch=True, mu_scale=0.85, delay_steps=2),
}
def job(args):
    scen, ctrl, case, seed = args
    cfg = default_config(); cfg['robust'].update(CASES[case]); cfg['robust']['seed'] = seed
    r = run_scenario(cfg, scen, ctrl); m = metrics(r); m.update(scen=scen, ctrl=ctrl, case=case, seed=seed); return m
if __name__ == '__main__':
    jobs = [(s, c, k, 1) for s in ['lane_change','s_curves','hairpin_wet','obstacles'] for c in CONTROLLERS for k in CASES]
    t0 = time.time()
    with Pool(int(sys.argv[1]) if len(sys.argv)>1 else 4) as p: res = p.map(job, jobs)
    json.dump(res, open('bench.json','w'))
    print('runs', len(res), 'wall', round(time.time()-t0))
