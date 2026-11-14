"""Frontier sweep: each configuration run over the budget scale ALPHA (the common conservatism knob),
giving one frontier of excess loss vs energy per configuration. Used by paper_sweep.py,
attainable_table.py and interval_wnm.py; results go to opt_out/<name>.json as rows of
{family, label, cfg, chan, summary}.
"""
import json
import os

from opt_run import OUT, run, summary

ALPHAS = [0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1]


def families(groups):
    for g in groups:
        for label, cfg in g.items():
            yield label, cfg


def sweep(name, chan, groups, seeds, alphas=ALPHAS, extra=None):
    cfgs, meta = [], []
    for label, cfg in families(groups):
        for a in alphas:
            c = dict(cfg, ALPHA=a, **(extra or {}))
            cfgs.append(c)
            meta.append((label, c))
    res = run(cfgs, chan=chan, seeds=seeds)
    rows = []
    for (label, c), r in zip(meta, res):
        rows.append(dict(family=label, cfg=c, chan=chan, summary=summary(r, float(chan["L"]), float({**chan, **c}.get("EXWIN", 300))),
                         hours=sum(x["nwin"] for x in r) * 300 / 3600))
    with open(os.path.join(OUT, name + ".json"), "w") as f:
        json.dump(rows, f, indent=1)
    return rows
