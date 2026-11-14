"""The loss debt's horizon on the final firmware (2026-11-03).

Round 2 (k_sweep_round2.py) was run on the controller with the fixed probe and feedback cycle and
without failed TCP sends charged to the debt. This repeats its R rows on the final firmware (random
probe floor, FW_DEBT_TCP=1) with round 2's scenarios and selection rule unchanged, on fresh
realizations (SEED0 = 20000). The k rows cannot be repeated: FW_DEBT_K was removed from the
controller after commit 36984c1a.

    python3 k_sweep_final.py
"""
import k_sweep as ks
from ge_channels import FINAL

SEED0 = 20000
ks.VARIANTS = {"no debt": dict(EST="fw", **FINAL)}
ks.VARIANTS.update({f"R={r}": dict(EST="fw", FW_DEBT_R=r, **FINAL) for r in (150, 200, 250, 300, 400, 500)})
ks.VARIANTS["oracle"] = dict(EST="oracle", **FINAL)
for cfg in ks.VARIANTS.values():
    cfg["SEED0"] = SEED0          # in the configuration, so the cache keys are new too

if __name__ == "__main__":
    ks.main()
