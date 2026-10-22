#!/usr/bin/env bash
# ============================================================
# run-comparison.sh — drive all 240 ns-3 simulations.
#
# Companion code for: Performance Evaluation of Cluster-Based WSN-AODV and
#          MQTT-SN Architectures for Medical IoT Using ns-3, A. Baseet and İ. Bütün.
# Repo:    https://github.com/amirbaseet/ns3-miot-simulation
#
# Usage:
#   bash scripts/run-comparison.sh                # Sets 1, 2, 3, 3b  (240 runs)
#
# Prerequisites:
#   * ns-3 v3.40 built under ~/ns-3-dev  (v3.41 causes PDR=0% — DO NOT USE)
#   * Scratch targets cluster-aodv-mqtt and cluster-aodv-nosink installed
#     (see README.md §Build and REPRODUCE.md)
#   * tmux strongly recommended: ≈ 8 h on a 4-core machine.
#
# Sets produced:
#   1  — No broker              (WSN vs MQTT-SN, 50/100/150/200 nodes)
#   2  — With broker            (WSN vs MQTT-SN, 50/100/150/200 nodes)
#   3  — Equal load, no broker  (WSN-Hetero vs MQTT-SN)
#   3b — Equal load, with broker
#
# Output:  ~/ns-3-dev/experiments/statistical/
# ============================================================
set -uo pipefail

# --- Argument parsing ---
case "${1:-}" in
    -h|--help)        sed -n '2,25p' "$0"; exit 0 ;;
    "")               ;;   # default: run everything
    *)                echo "Unknown argument: $1"; echo "Use --help for usage."; exit 2 ;;
esac

cd ~/ns-3-dev
mkdir -p experiments/statistical

SIMTIME=300
SEEDS=10

echo ""
echo "============================================================"
echo "  WSN vs MQTT-SN"
echo "  Seeds: $SEEDS | SimTime: ${SIMTIME}s"
echo "============================================================"

# ============================================================
# SET 1: NO BROKER
# ============================================================
echo ""
echo "======== SET 1: NO BROKER ========"
for N in 50 100 150 200; do
    CH=$((N / 10))
    for RUN in $(seq 1 $SEEDS); do
        echo "[Set1-WSN] N=$N Run=$RUN..."
        ./ns3 run "cluster-aodv-nosink --numRegular=$N --numCH=$CH --useSink=false --run=$RUN --simTime=$SIMTIME --csv=experiments/statistical/exp1_wsn_${N}_r${RUN}.csv" 2>/dev/null
        echo "[Set1-MQTT] N=$N Run=$RUN..."
        ./ns3 run "cluster-aodv-mqtt --nSensors=$N --nCH=$CH --numLocalBrokers=0 --run=$RUN --simTime=$SIMTIME --anim=false --csv=experiments/statistical/exp1_mqtt_${N}_r${RUN}.csv" 2>/dev/null
    done
    echo "[OK] Set1 N=$N complete"
done

# ============================================================
# SET 2: WITH BROKER (single)
# ============================================================
echo ""
echo "======== SET 2: WITH BROKER ========"
for N in 50 100 150 200; do
    CH=$((N / 10))
    for RUN in $(seq 1 $SEEDS); do
        echo "[Set2-WSN] N=$N Run=$RUN..."
        ./ns3 run "cluster-aodv-nosink --numRegular=$N --numCH=$CH --useSink=true --run=$RUN --simTime=$SIMTIME --csv=experiments/statistical/exp2_wsn_${N}_r${RUN}.csv" 2>/dev/null
        echo "[Set2-MQTT] N=$N Run=$RUN..."
        ./ns3 run "cluster-aodv-mqtt --nSensors=$N --nCH=$CH --numLocalBrokers=1 --run=$RUN --simTime=$SIMTIME --anim=false --csv=experiments/statistical/exp2_mqtt_${N}_r${RUN}.csv" 2>/dev/null
    done
    echo "[OK] Set2 N=$N complete"
done

# ============================================================
# SET 3: EQUAL LOAD, NO BROKER
# ============================================================
echo ""
echo "======== SET 3: EQUAL LOAD, NO BROKER ========"
for N in 50 100 150 200; do
    CH=$((N / 10))
    for RUN in $(seq 1 $SEEDS); do
        echo "[Set3-WSN-Hetero] N=$N Run=$RUN..."
        ./ns3 run "cluster-aodv-nosink --numRegular=$N --numCH=$CH --useSink=false --hetero=true --run=$RUN --simTime=$SIMTIME --csv=experiments/statistical/exp3_wsn_hetero_${N}_r${RUN}.csv" 2>/dev/null
    done
    echo "[OK] Set3 N=$N complete"
done

# ============================================================
# SET 3b: EQUAL LOAD, WITH BROKER
# ============================================================
echo ""
echo "======== SET 3b: EQUAL LOAD, WITH BROKER ========"
for N in 50 100 150 200; do
    CH=$((N / 10))
    for RUN in $(seq 1 $SEEDS); do
        echo "[Set3b-WSN-Hetero] N=$N Run=$RUN..."
        ./ns3 run "cluster-aodv-nosink --numRegular=$N --numCH=$CH --useSink=true --hetero=true --run=$RUN --simTime=$SIMTIME --csv=experiments/statistical/exp3b_wsn_hetero_broker_${N}_r${RUN}.csv" 2>/dev/null
    done
    echo "[OK] Set3b N=$N complete"
done

echo ""
echo "============================================================"
echo "  ALL EXPERIMENTS COMPLETE!"
echo "============================================================"
echo "Statistical CSVs:"
ls experiments/statistical/*.csv 2>/dev/null | wc -l
echo "total CSV files."
