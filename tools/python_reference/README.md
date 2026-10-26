# Python reference implementation

A Python mirror of the MATLAB code (same vehicle model, controllers,
scenarios and robustness cases). It produced the reference results in the
main README, and is useful for checking the MATLAB version or running
experiments without MATLAB.

```bash
pip install numpy scipy
python run_benchmark.py 4        # 4 worker processes; writes bench.json
```

```python
from sim import default_config, run_scenario, metrics
cfg = default_config()
res = run_scenario(cfg, 'hairpin_wet', 'nmpc')
print(metrics(res))
```
