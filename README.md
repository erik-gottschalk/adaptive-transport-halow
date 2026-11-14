# Adaptive transport layer for Wi-Fi HaLow

Code accompanying the paper

> E. Gottschalk, M. N. Petersen, S. Ruepp, M. S. Berger, *Paying Only for the Reliability You Need:
> An Adaptive Transport Layer for Energy-Constrained IoT* (under review).

The adaptive transport layer sends each message of an IoT device over UDP or TCP so that every
message class meets the loss tolerance the application declares for it, at minimum energy, from
observed delivery outcomes alone. This repository holds the firmware that runs it on a Wi-Fi HaLow
(IEEE 802.11ah) devkit, the testbed scripts that measured it, a simulator that compiles the
firmware's controller unmodified, and the scripts behind every table and figure of the paper. The
measurement data is a separate repository, included here as a submodule.

## Contents

The repository is Morse Micro's [MM-IoT-SDK](https://github.com/MorseMicro/mm-iot-sdk) 2.13.1
(its history up to that release, unchanged) with one commit on top: the adaptive transport layer in
[`examples/adaptive_transport/`](examples/adaptive_transport/), and a small change to the SDK's
`mm-mm8108-ekh05` platform:

| Path | What |
|---|---|
| `src/adaptive_transport_controller.{c,h}` | the controller: estimator, transport plan, loss debt, feedback protocol |
| `src/adaptive_transport.c` | the firmware application: periodic messages over UDP/TCP, debug pins, remote configuration |
| `scripts/server.py` | the receiver: counts each client's datagrams and answers delivery reports |
| `scripts/eval/run_paper.sh` | everything the paper reports: the hardware captures, the simulations, the analysis |
| `scripts/run_power_experiment.py` | one hardware capture |
| `scripts/channel_scheduler.py`, `scripts/make_ge_schedule.py` | loss injected at the access point, and the bursty schedule |
| `scripts/eval/analysis/` | the analysis of the captures, and the paper's hardware figures and numbers |
| `scripts/eval/sim/` | the simulator and the paper's simulations |
| `scripts/eval/measurement_data/` | the measurement data (submodule) |

The SDK's support for this devkit has two debug pins, `MM_DEBUG_0` (PA10) and `MM_DEBUG_1` (PA9);
the platform change adds a third, `MM_DEBUG_2` (PA0). For the current measurement, the firmware
marks every send and its transport on PA10 and PA0 (the Power Profiler's D0 and D1 inputs) and
holds PA9 high as the logic-high reference for those inputs, since the devkit has no 3.3 V pin near
the debug header. The larger lwIP connection and segment pools the firmware needs are set in the
example's `app.mk`.

## Getting started

```sh
git clone --recursive <URL of this repository>
cd adaptive-transport-halow
python3 -m venv .venv && .venv/bin/pip install -r examples/adaptive_transport/scripts/requirements.txt
```

Without `--recursive` (or when downloading a ZIP from GitHub) the data folder is empty; fetch it
with `git submodule update --init`.

**Build the firmware** for the Morse Micro `mm-mm8108-ekh05` devkit (the SDK's toolchain setup is
described in its user guide, `documentation.html`): copy
`examples/adaptive_transport/src/server_config.local.h.example` to `server_config.local.h`, set the
address of the host that runs `server.py`, then

```sh
cd examples/adaptive_transport/targets/mm-mm8108-ekh05 && make
```

and write the Wi-Fi settings of `examples/adaptive_transport/config.hjson` to the device's config
store as the SDK describes.

## Reproducing the paper

Every table, figure and number of the paper (main text and supplementary material) comes from the
published data or the simulator; no hardware is needed. Commands run from
`examples/adaptive_transport/scripts/eval/` in the repository's Python environment; data folders are
under `measurement_data/`, one per experiment (its README describes them). All of them at once:

```sh
./run_paper.sh analysis                       # every table and number below, and the figures into plots/
OPT_WORKERS=16 ./run_paper.sh simulations     # every simulation, into sim/opt_out/
```

The hardware analyses take seconds to minutes. The simulations take about 20 minutes with
`OPT_WORKERS=16` (on a 32-core machine, from an empty cache) and reproduce the outputs committed in
`sim/opt_out/rerun_window_ge/` and `sim/opt_out/t3_mac_retry*` byte for byte; results are cached in `sim/opt_out/`, so a rerun only
simulates what is missing. Every analysis reads a capture's raw current trace if it is present and
its cycles file otherwise, with identical results; the few results that read samples within a cycle
are stored in `measurement_data/derived/`.

### Figures

`python analysis/export_ieee_figures.py` draws Fig. 3 to 7 into `plots/` (`--out <dir>` elsewhere,
`--only steady,step,mixed,traces,bursty` a selection). Fig. 1 and 2 are drawings.

| Figure | `--only` | Data |
|---|---|---|
| Fig. 3 steady-state grid | `steady` | `baselines`, `grid/debt`, `grid/binding_cell` (L = 5 %, p = 20 %), `grid/fixed_probe` (L = 0) |
| Fig. 4 step schedule | `step` | `step` |
| Fig. 5 mixed workload | `mixed` | `mixed` |
| Fig. 6 packet traces | `traces` | `baselines` r01 at p = 0 (`derived/example_traces.json`) |
| Fig. 7 bursty, changing loss | `bursty` | `bursty/baselines` (UDP, TCP), `bursty/controller` |

### Tables

| Table | Command | Data |
|---|---|---|
| I energy per message, G, C | `python sim/hw_table.py` (E_UDP, E_TCP^0, E_TCP^r per p) | `baselines` |
| I acknowledged UDP | `python analysis/analyze_udp_ack.py` | `udp_ack` |
| II bursty, changing loss | `python analysis/analyze_ge_schedule.py`; UDP's ">1.5L": `python analysis/t1_bursty_over_15L.py` | `bursty` |
| III design choices | `python sim/design_table.py` | simulation |
| S1 notation | — | — |
| S2 deep sleep | `python analysis/analyze_deep_sleep.py -o /dev/null` | `deep_sleep` |
| S3 sleep share | first column: `analysis/analyze_deep_sleep.py`; controller: `python sim/interval_wnm.py` | simulation |
| S4 MAC retries | `python sim/t3_mac_retry.py` (recorded in `sim/opt_out/t3_mac_retry.log`); the transmit bursts per message it builds on: `python analysis/t3_frames_from_traces.py` | simulation; `baselines` r01 at p = 0 (`derived/t3_frames.json`) |
| S5 steady overload, hw vs sim | hardware: `python analysis/grid_table.py`; simulation: `python sim/steady_table.py` | `grid` |
| S6 design choices (full) | `python sim/design_table.py`; the loss/L* columns: `python sim/attainable_table.py` | simulation |
| S7 channels beyond the testbed | `python sim/attainable_table.py` | simulation |
| S8 half-lives | `python sim/attainable_table.py` (bursty columns), `python sim/sched20_paper.py` (step columns) | simulation |
| S9 loss debt's horizon | `python sim/k_sweep_final.py` | simulation |

The simulator compiles `src/adaptive_transport_controller.c` unmodified (`sim/build.sh`; needs gcc)
and takes its energy and timing from the hardware tables `sim/hw_table.txt` and
`sim/hw_table_wnm.txt`, which `sim/hw_table.py` and `analysis/analyze_deep_sleep.py` write.

### Numbers in the text

| Number | Command |
|---|---|
| controller footprint (3.6 KB code, 440 B state) | build the firmware, then `../footprint.sh` |
| microcontroller overhead of the adaptive firmware (at most 0.012 mW more than plain TCP, below 0.1 % of the HaLow module's per-message energy) | measured separately at the devkit's microcontroller current tap; not part of the published captures |
| steady state: binding cells' mean loss/L, margin at p = L, savings at p = 0 and the largest | `python analysis/grid_table.py` |
| smallest saving of the 31 adaptive captures, worst-case G/C, TCP losses under independent loss, largest feedback window, awake period after a send, bursty binding cell's plain TCP, capture count | `python analysis/paper_numbers.py` |
| step schedule: phase loss/L, reaction, whole-schedule loss and saving | `python analysis/analyze_step_schedule.py` |
| mixed workload: per-class loss/L, Wilson intervals, UDP share | `python analysis/analyze_mixed_criticality.py` |
| mixed workload: reports carried per class | `python analysis/mixed_reports.py measurement_data/mixed` |
| acknowledged UDP: energies, acknowledgment delay, retransmissions | `python analysis/analyze_udp_ack.py` |
| deep sleep: wake-up cost, G, C | `python analysis/analyze_deep_sleep.py -o /dev/null` |
| bursty schedule: L*, reaction to jumps, saving, share of the bound | `python analysis/analyze_ge_schedule.py` |
| bursty binding cell (controller) | `python analysis/analyze_ge_schedule.py binding` |
| replies that miss the in-line wait (1.4, 4.0 and 11 % at p = 5, 10 and 20 %) | `python sim/hw_table.py` (column `late`) |
| simulated binding cell under bursts | `python sim/binding_cell_final.py` |
| alarm class against run length | `python sim/mix_alarm_run_length.py` |
| windows above 1.5L with and without the debt | `sim/opt_out/rerun_window_ge/k_sweep_final.txt` (`>1.5L(5min)`, R=250 and no debt) |
| long reporting intervals | `python sim/attainable_table.py` (`== message interval`), `sim/interval_wnm.py` |

## Testbed

The device under test is a Morse Micro `mm-mm8108-ekh05` devkit (STM32 microcontroller, MM8108
HaLow module) associated to a Morse Micro HaLowLink 2 access point. Messages travel over the
internet to `server.py`. A Nordic Power Profiler Kit II measures the HaLow module's current at the
devkit's current tap at 10 kHz, together with two debug pins that mark every send and its
transport. Channel loss is injected at the access point, at the network layer: a packet-filter rule
(nftables) drops packets independently with probability *p*, and the network emulator (netem) runs
a Gilbert–Elliott chain for bursty loss. The radio link itself loses no packets on the bench.

`./run_paper.sh captures` (in `scripts/eval/`) makes every capture of the paper again, about 6 days
of measurement, or `./run_paper.sh <experiment>` those of one: `baselines`, `grid`, `step`,
`mixed`, `bursty`, `udp_ack`, `deep_sleep` (`./run_paper.sh --help` says what each covers;
`DRY_RUN=1` prints the plan, one line per capture). It runs the published captures' cells,
replicates, loss schedules and controller settings, into the same folders, and builds and flashes
the firmware itself. It reads the lab's addresses and serial ports from `scripts/lab.local.env`
(template: `scripts/lab.env.example`).

## Data

[`scripts/eval/measurement_data/`](examples/adaptive_transport/scripts/eval/measurement_data/) (a
separate repository) holds every capture the paper uses: run manifests, device consoles, receiver
logs, the injected loss over time, and per send the cycle from the current trace from which every
energy figure is computed. The raw current traces (10 kHz, 55 GB for the 188 captures) are not
included; they are available from the corresponding author on request, and `TRACES.sha256` in the
data repository identifies them. Every published capture behaves as this firmware with the build
options and run settings that `captures.csv` there lists for it; see "Firmware and settings" in its
README.

## Notes on the code

- The warning `seq dropped from <n> to 0 ... possible device restart`, printed for most captures,
  is expected: the receiver's log starts before the device reboots for the capture, so it holds
  late messages of the previous run and then the restart of the sequence numbers. In every
  published capture this restart lies before the capture window, which is all the analysis uses.
- `scripts/eval/analysis/power_common.py` documents the energy accounting in detail: what a send's
  cycle covers, why protocol classification uses the debug pins and not pulse widths, and how
  captures are aligned with the receiver's log.

## License

The MM-IoT-SDK keeps its licenses (see `LICENSES/` and the headers of its files). The license of
the adaptive transport layer's code and data will be added with the paper's publication.

## About the MM-IoT-SDK

This package contains the Morse Micro IoT SDK, providing a framework for getting started with Morse
Micro HaLow Wi-Fi. Open
[documentation.html](https://morsemicro.github.io/mm-iot-sdk/documentation.html) for the full
MM-IoT-SDK user guide. This repository and its contents, including software component Makefiles
and metadata, are licensed under the Apache 2.0, unless otherwise stated in individual files.
Software components from third party sources provide their own licenses.
