# ns3-miot-simulation (code only)

ns-3 simulation code and analysis script for:

> A. Baseet and İ. Bütün, "Performance Evaluation of Cluster-Based WSN-AODV
> and MQTT-SN Architectures for Medical IoT Using ns-3."

This repository contains source code and scripts only. It contains no
simulation outputs (CSV files) and no run logs.

## Contents

| Path | Purpose |
|---|---|
| `src/wsn/cluster-aodv-nosink.cc` | Architecture A: cluster-based WSN with AODV (optional sink, optional equal-load traffic) |
| `src/mqtt-sn/cluster-aodv-mqtt.cc` | Architecture B: MQTT-SN scenario (no broker, single broker, or two-tier hierarchy) |
| `src/mqtt-sn/mqtt-sn-{header,publisher,gateway,broker}.{h,cc}` | MQTT-SN protocol components |
| `scripts/run-comparison.sh` | Runs Sets 1, 2, 3 and 3b (240 runs) |
| `scripts/analyze_comparison.py` | Computes per-configuration mean ± std, Welch t-tests, Mann–Whitney U tests, and figures |
| `scripts/generate_topology_fig.py` | Draws the topology figure |
| `requirements.txt` | Python packages for the analysis |

## Requirements

- Linux (re-runs have been checked on Kali Linux)
- **ns-3 version 3.40** (git tag `ns-3.40`). Other ns-3 versions have not
  been validated with this code.
- C++ toolchain, CMake, Ninja, Python 3
- Python packages: `pip install -r requirements.txt`
  (numpy, pandas, scipy, matplotlib)

## 1. Build

```bash
# ns-3.40
cd ~
git clone https://gitlab.com/nsnam/ns-3-dev.git
cd ns-3-dev
git checkout ns-3.40
git describe --tags --exact-match      # must print: ns-3.40

# Install the two scenarios as ns-3 scratch programs
mkdir -p scratch/cluster-aodv-mqtt scratch/cluster-aodv-nosink
cp <this-repo>/src/mqtt-sn/*                  scratch/cluster-aodv-mqtt/
cp <this-repo>/src/wsn/cluster-aodv-nosink.cc scratch/cluster-aodv-nosink/

./ns3 configure
./ns3 build
```

`scripts/run-comparison.sh` expects ns-3 at `~/ns-3-dev` and writes its
output there.

## 2. Seeds

Both scenarios call `SeedManager::SetSeed(42)` and
`SeedManager::SetRun(run)`, where `run` is the `--run` argument. Every
configuration is run with `--run=1` to `--run=10` (10 independent
replications). The simulated time is `--simTime=300` (seconds). The
number of cluster heads is one per 10 sensors (`numCH = N / 10`).

## 3. Run the experiments

All sets:

```bash
cd ~/ns-3-dev
bash <this-repo>/scripts/run-comparison.sh            # Sets 1, 2, 3, 3b
```

Use `tmux` or `nohup`: a full run takes many hours. Measured on one
200-node run each (6-core aarch64 VM, ns-3.40): a Set 2 WSN run took
5,256 s. Runs at fewer nodes are shorter.
Each run uses one CPU core, so several runs can be started in parallel.

The commands per set (N ∈ {50, 100, 150, 200}, CH = N/10, RUN ∈ 1..10):

| Set | Description | Command (inside `~/ns-3-dev`) | Output file |
|---|---|---|---|
| 1 | No broker, WSN | `./ns3 run "cluster-aodv-nosink --numRegular=N --numCH=CH --useSink=false --run=RUN --simTime=300 --csv=..."` | `experiments/statistical/exp1_wsn_N_rRUN.csv` |
| 1 | No broker, MQTT-SN | `./ns3 run "cluster-aodv-mqtt --nSensors=N --nCH=CH --numLocalBrokers=0 --run=RUN --simTime=300 --anim=false --csv=..."` | `experiments/statistical/exp1_mqtt_N_rRUN.csv` |
| 2 | Single broker, WSN | `cluster-aodv-nosink ... --useSink=true` | `experiments/statistical/exp2_wsn_N_rRUN.csv` |
| 2 | Single broker, MQTT-SN | `cluster-aodv-mqtt ... --numLocalBrokers=1` | `experiments/statistical/exp2_mqtt_N_rRUN.csv` |
| 3 | Equal load, no broker (WSN-Hetero) | `cluster-aodv-nosink ... --useSink=false --hetero=true` | `experiments/statistical/exp3_wsn_hetero_N_rRUN.csv` |
| 3b | Equal load, with broker (WSN-Hetero) | `cluster-aodv-nosink ... --useSink=true --hetero=true` | `experiments/statistical/exp3b_wsn_hetero_broker_N_rRUN.csv` |

Expected file counts: Set 1 = 80, Set 2 = 80, Set 3 = 40, Set 3b = 40,
total 240.

Sets 3 and 3b simulate only the WSN-Hetero side. Their MQTT-SN comparison
columns reuse Set 1 (no broker) and Set 2 (with broker) MQTT-SN data.

Each CSV has one row per IPv4 flow (ns-3 FlowMonitor) with the columns
`FlowID, SrcAddr, DstAddr, TxPackets, RxPackets, LostPackets, PDR_pct,
Throughput_kbps, AvgDelay_ms, AvgJitter_ms`.

## 4. Reproduce the tables

```bash
cd ~/ns-3-dev
python3 <this-repo>/scripts/analyze_comparison.py | tee analysis_output.txt
```

The script reads the CSVs from `experiments/statistical/` relative to the
current directory. It prints, for each
configuration, mean ± standard deviation of PDR and end-to-end delay, the
Welch t-test p-value, and the Mann–Whitney U p-value. Figures are written to
`graphs/comparison/`.

How the reported values are computed:

- **Sensor-origin flows only.** PDR and delay use flows whose source
  address has a last octet from 1 to N (the sensors). Cluster-head,
  broker, and acknowledgment flows are excluded.
- **PDR** = sum of RxPackets / sum of TxPackets over those flows, per run.
- **Delay** = mean of AvgDelay_ms over those flows with a delay above 0, per run.
- **Brokered 200-node first batch.** For Set 2 at 200 nodes the script
  reads seeds 1–6 (WSN) and 1–5 (MQTT-SN) only (`FIRST_BATCH` in
  `analyze_comparison.py`). See the paper, Section V-D.

## License

MIT. See `LICENSE`.
