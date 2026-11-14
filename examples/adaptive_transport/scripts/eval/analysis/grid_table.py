"""Values of the paper's steady-state grid (Fig. 3, the hardware columns of Table S5
and the numbers quoted in Section IV-C1), from the same cells as
Fig. 3 (export_ieee_figures.py): with the loss debt grid/debt (random probe), without it
grid/no_debt (ATC_DEBT_R = 0) where the budget binds, grid/fixed_probe (fixed probe cycle, no debt)
below that; L = 0 from grid/fixed_probe;
p <= 20 %. The v3 no-debt value is printed alongside for comparison.

    python3 grid_table.py

Per cell: delivered loss / L, and energy saving against plain TCP at the same injected loss, per
message sent (Fig. 3, Table S5) and per delivered message (the remark in
Section IV-E), each from the cell means of the adaptive and TCP captures (TCP
interpolated at p = 1 %).
"""
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
DATA = HERE.parent / "measurement_data"
sys.path.insert(0, str(HERE))
import summarize_pwr_runs as spr  # noqa: E402

BASELINE_DIRS = [DATA / "baselines"]
P_MAX = 20.0


def cell_of(path):
    m = json.loads(path.read_text())
    return round(float(m["L"]), 4), round(float(m["schedule"][0][1]), 4)


def runs(campaign):
    out = {}
    for p in sorted((DATA / campaign).glob("pwr_adp_*.json")):
        c = cell_of(p)
        if c[1] <= P_MAX:
            out.setdefault(c, []).append(p)
    return out


cache = spr.load_cache()
base = sorted(m for d in BASELINE_DIRS for m in d.glob("pwr_tcp_L*.json"))   # not pwr_tcp_reply_*
tcp_cells = {}
for m in base:
    r = spr.process_one(m, cache)
    if "error" not in r:
        tcp_cells.setdefault(cell_of(m)[1], []).append(r["out"]["energy"])
tcp_p = sorted(tcp_cells)
TCP = {"mJ/pkt_delivered_mean": [np.mean([e["mJ_per_packet_delivered"] for e in tcp_cells[q]]) for q in tcp_p],
       "mJ/pkt_attempted_mean": [np.mean([e["mJ_per_packet_attempted"] for e in tcp_cells[q]]) for q in tcp_p]}


def tcp_at(p, col):
    return float(np.interp(p, tcp_p, TCP[col]))


def cell(paths):
    rows = [spr.process_one(p, cache) for p in paths]
    rows = [r for r in rows if "error" not in r]
    # The quality gates of summarize_pwr_runs.cell_aggregate: power and loss are pooled separately.
    pw = [r for r in rows if not spr.excluded(r, spr.POWER_INVALIDATES)]
    ad = [r for r in rows if not spr.excluded(r, spr.ADAPTIVE_INVALIDATES)]
    for r in rows:
        if r not in pw or r not in ad:
            print(f"  QA excludes {r['basename']} from {'energy' if r not in pw else ''} "
                  f"{'loss' if r not in ad else ''}")
    L = rows[0]["adaptive"]["L_target"]
    loss = np.mean([r["adaptive"]["delivered_loss"] for r in ad])
    e_del = np.mean([r["out"]["energy"]["mJ_per_packet_delivered"] for r in pw])
    e_att = np.mean([r["out"]["energy"]["mJ_per_packet_attempted"] for r in pw])
    return dict(n=len(ad), loss_L=loss / L if L else np.nan, loss=loss, e_del=e_del, e_att=e_att,
                runs=[r["adaptive"]["delivered_loss"] / L if L else np.nan for r in ad])


v5, v5nd, v3 = runs("grid/debt"), runs("grid/no_debt"), runs("grid/fixed_probe")
# The L = 5 %, p = 20 % cell rerun with failed sends charged (grid/binding_cell), in place of debt's, as
# in the steady-state figure (export_ieee_figures.steady) and the IEEE version's tables.
final = {}
for p in sorted((DATA / "grid" / "binding_cell").glob("pwr_adp_*.json")):
    final.setdefault(cell_of(p), []).append(p)
v5 = {**v5, **final}
nodebt = {**v3, **v5nd}            # v5 without the debt wherever it was measured
res = {}
for c in sorted(set(v5) | set(v3)):
    res[c] = {k: cell(src[c]) for k, src in (("debt", v5), ("nodebt", nodebt), ("v3", v3)) if c in src}
spr.save_cache(cache)

print("steady-state grid  (loss/L no debt, debt | saving per delivered no debt, debt | per sent no debt, debt | n)")
print("             the table uses per sent; per delivered is quoted in the text only")
for (L, p), d in res.items():
    def f(k, key, fmt):
        return fmt.format(d[k][key]) if k in d else "--"

    def sav(k, which):
        if k not in d:
            return "--"
        e = d[k]["e_del" if which == "del" else "e_att"]
        ref = tcp_at(p, "mJ/pkt_delivered_mean" if which == "del" else "mJ/pkt_attempted_mean")
        return f"{100 * (1 - e / ref):.1f}%"
    print(f"  L={100 * L:4.1f}% p={p:4.0f}%  {f('nodebt', 'loss_L', '{:.2f}'):>5} {f('debt', 'loss_L', '{:.2f}'):>5} | "
          f"{sav('nodebt', 'del'):>6} {sav('debt', 'del'):>6} | {sav('nodebt', 'att'):>6} {sav('debt', 'att'):>6} | "
          f"n={d.get('nodebt', {}).get('n', 0)}/{d.get('debt', {}).get('n', 0)}  "
          f"no-debt src {'v5' if (L, p) in v5nd else 'v3'} (v3 {f('v3', 'loss_L', '{:.2f}')})  "
          f"debt runs {' '.join(f'{x:.2f}' for x in d.get('debt', {}).get('runs', []))}"
          + (f"  L=0 loss {d['nodebt']['loss']:.4f}, energy vs TCP {100 * (d['nodebt']['e_att'] / tcp_at(p, 'mJ/pkt_attempted_mean') - 1):+.1f}%"
             if L == 0 else ""))

bind = [(c, d) for c, d in res.items() if c[0] > 0 and c[1] > 100 * c[0] and "debt" in d and "nodebt" in d]
print("\nbinding cells p > L:", len(bind))
print(f"  no debt loss/L {min(d['nodebt']['loss_L'] for _, d in bind):.2f}-{max(d['nodebt']['loss_L'] for _, d in bind):.2f}, "
      f"mean {np.mean([d['nodebt']['loss_L'] for _, d in bind]):.2f}")
print(f"  debt    loss/L {min(d['debt']['loss_L'] for _, d in bind):.2f}-{max(d['debt']['loss_L'] for _, d in bind):.2f}, "
      f"mean {np.mean([d['debt']['loss_L'] for _, d in bind]):.2f}")
for e, col, name in (("e_att", "mJ/pkt_attempted_mean", "per sent"), ("e_del", "mJ/pkt_delivered_mean", "per delivered")):
    cost = [100 * (d['debt'][e] - d['nodebt'][e]) / tcp_at(c[1], col) for c, d in bind]
    print(f"  saving given back by the debt ({name}): mean {np.mean(cost):.1f} points, "
          f"per cell {' '.join(f'{x:+.1f}' for x in cost)}")
eq = [(c, d) for c, d in res.items() if c[0] > 0 and abs(c[1] - 100 * c[0]) < 1e-6]
print("p = L cells: " + "  ".join(f"L={100 * c[0]:g}%: no debt {d.get('nodebt', {}).get('loss_L', np.nan):.2f}, debt {d['debt']['loss_L']:.2f}" for c, d in eq))
p0 = [(c, d) for c, d in res.items() if c[0] > 0 and c[1] == 0 and "debt" in d]
for e, col, name in (("e_att", "mJ/pkt_attempted_mean", "per sent"), ("e_del", "mJ/pkt_delivered_mean", "per delivered")):
    print(f"p = 0 savings {name}: " + "  ".join(
        f"L={100 * c[0]:g}%: {100 * (1 - d['debt'][e] / tcp_at(0, col)):.1f}%" for c, d in p0))
gap = [100 * ((1 - d['debt']['e_att'] / tcp_at(c[1], 'mJ/pkt_attempted_mean'))
              - (1 - d['debt']['e_del'] / tcp_at(c[1], 'mJ/pkt_delivered_mean')))
       for c, d in res.items() if c[0] > 0 and 'debt' in d]
print(f"saving per sent minus per delivered, debt cells L > 0: {min(gap):.1f}-{max(gap):.1f} points, mean {np.mean(gap):.1f}")
print(f"TCP at p = 0: {tcp_at(0, 'mJ/pkt_attempted_mean'):.2f} mJ per sent, {tcp_at(0, 'mJ/pkt_delivered_mean'):.2f} per delivered")
