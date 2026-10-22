"""
fig_topology.py
Two-panel system topology figure.
Companion code for: Performance Evaluation of Cluster-Based WSN-AODV and
MQTT-SN Architectures for Medical IoT Using ns-3, A. Baseet and İ. Bütün.
  (a) WSN-AODV
  (b) MQTT-SN publish-subscribe
"""
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyArrowPatch, FancyBboxPatch, Rectangle
from matplotlib.lines import Line2D

plt.rcParams.update({
    "font.family": "serif",
    "font.serif": ["Times New Roman", "Liberation Serif", "DejaVu Serif"],
    "font.size": 8.5,
    "axes.linewidth": 0.6,
    "lines.linewidth": 0.8,
    "pdf.fonttype": 42,
    "ps.fonttype": 42,
})

COL_SENSOR, COL_ECG, COL_HR, COL_TEMP = "#4a7ab8", "#c0392b", "#e67e22", "#27ae60"
COL_CH, COL_SINK = "#2c3e50", "#6c3483"
COL_ARROW, COL_DASHED, COL_PDR, COL_FRAME = "#555555", "#7f8c8d", "#b71c1c", "#cccccc"

def sensor(ax, x, y, color=COL_SENSOR, r=0.28):
    ax.add_patch(Circle((x, y), r, facecolor=color, edgecolor="black",
                        linewidth=0.5, zorder=4))

def node_box(ax, cx, cy, w, h, label, sublabel=None, color=COL_CH):
    box = FancyBboxPatch((cx - w/2, cy - h/2), w, h,
                         boxstyle="round,pad=0.02,rounding_size=0.12",
                         facecolor=color, edgecolor="black",
                         linewidth=0.7, zorder=4)
    ax.add_patch(box)
    if sublabel is None:
        ax.text(cx, cy, label, ha="center", va="center",
                color="white", fontsize=8.2, fontweight="bold", zorder=5)
    else:
        ax.text(cx, cy + h*0.22, label, ha="center", va="center",
                color="white", fontsize=8, fontweight="bold", zorder=5)
        ax.text(cx, cy - h*0.22, sublabel, ha="center", va="center",
                color="white", fontsize=6.7, zorder=5)

def arrow(ax, x1, y1, x2, y2, color=COL_ARROW, lw=0.8,
          dashed=False, mutation=8, shrinkA=3, shrinkB=4):
    style = "--" if dashed else "-"
    a = FancyArrowPatch((x1, y1), (x2, y2),
                        arrowstyle="-|>", mutation_scale=mutation,
                        color=color, linewidth=lw, linestyle=style,
                        zorder=2, shrinkA=shrinkA, shrinkB=shrinkB)
    ax.add_patch(a)

def pdr_marker(ax, x, y, label_suffix, side="right"):
    dx = 0.85 if side == "right" else -0.85
    ha = "left" if side == "right" else "right"
    tx_gap = 0.18 if side == "right" else -0.18
    ax.add_patch(Circle((x + dx, y), 0.13, facecolor="white",
                        edgecolor=COL_PDR, linewidth=1.0, zorder=5))
    ax.add_patch(Circle((x + dx, y), 0.05, facecolor=COL_PDR,
                        edgecolor=COL_PDR, zorder=6))
    ax.text(x + dx + tx_gap, y + 0.22, "PDR", color=COL_PDR,
            fontsize=6.8, fontweight="bold", ha=ha, va="center", zorder=6)
    ax.text(x + dx + tx_gap, y - 0.22, label_suffix, color=COL_PDR,
            fontsize=6.3, style="italic", ha=ha, va="center", zorder=6)

def area_frame(ax, title):
    rect = Rectangle((0.25, 0.25), 9.5, 9.5, facecolor="none",
                     edgecolor=COL_FRAME, linewidth=0.7,
                     linestyle=(0, (2, 2)), zorder=0)
    ax.add_patch(rect)
    ax.text(9.55, 0.45, "1000 m  x  1000 m  simulation area",
            fontsize=6.5, color="#888888", ha="right", va="bottom",
            style="italic")
    ax.text(5.0, 10.3, title, ha="center", va="center",
            fontsize=10, fontweight="bold")

def draw_panel_wsn(ax):
    area_frame(ax, "(a)  WSN-AODV  (Architecture A)")
    sensors = [(2.3, 8.0), (4.0, 8.6), (5.9, 8.6), (7.6, 8.0), (8.2, 5.6)]
    for (x, y) in sensors:
        sensor(ax, x, y, color=COL_SENSOR)
    ax.annotate("OnOffApplication\nUDP  4 kbps  /  512 B",
                xy=(2.3, 8.0), xytext=(0.7, 9.1),
                fontsize=6.8, ha="left", va="top",
                arrowprops=dict(arrowstyle="-", lw=0.4, color="#888888"))
    ch_cx, ch_cy = 5.0, 5.6
    node_box(ax, ch_cx, ch_cy, w=2.2, h=1.1,
             label="Cluster Head", sublabel="UdpForwarder", color=COL_CH)
    for (x, y) in sensors:
        arrow(ax, x, y, ch_cx, ch_cy + 0.55, lw=0.7, mutation=7)
    ax.text(5.0, 6.95, "uniform-rate UDP", fontsize=6.6, ha="center",
            color=COL_ARROW, style="italic")
    pdr_marker(ax, ch_cx + 1.1, ch_cy, "(Set 1)", side="right")
    sink_cx, sink_cy = 5.0, 2.3
    arrow(ax, ch_cx, ch_cy - 0.55, sink_cx, sink_cy + 0.55,
          color=COL_DASHED, dashed=True, lw=1.0, mutation=9,
          shrinkA=4, shrinkB=4)
    ax.text(ch_cx + 0.18, (ch_cy + sink_cy)/2 + 0.15,
            "optional\n(Set 2)", fontsize=6.7, color=COL_DASHED,
            ha="left", va="center", style="italic")
    node_box(ax, sink_cx, sink_cy, w=2.4, h=1.1,
             label="Central Sink", sublabel="aggregator", color=COL_SINK)
    pdr_marker(ax, sink_cx + 1.2, sink_cy, "(Set 2)", side="right")
    ax.set_xlim(0, 10); ax.set_ylim(0, 10.8)
    ax.set_aspect("equal"); ax.axis("off")

def draw_panel_mqttsn(ax):
    area_frame(ax, "(b)  MQTT-SN  (Architecture B)")
    sensors = [
        (2.3, 8.0, COL_ECG,  "ECG\nQoS 2"),
        (4.0, 8.6, COL_ECG,  "ECG\nQoS 2"),
        (5.9, 8.6, COL_HR,   "HR\nQoS 1"),
        (7.6, 8.0, COL_HR,   "HR\nQoS 1"),
        (8.2, 5.6, COL_TEMP, "Temp\nQoS 0"),
    ]
    for (x, y, c, lab) in sensors:
        sensor(ax, x, y, color=c, r=0.30)
        ax.text(x, y + 0.55, lab, fontsize=5.9, ha="center", va="bottom",
                color=c, fontweight="bold")
    ax.annotate("MqttSnPublisher",
                xy=(2.3, 8.0), xytext=(0.45, 9.45),
                fontsize=6.8, ha="left", va="top",
                arrowprops=dict(arrowstyle="-", lw=0.4, color="#888888"))
    gw_cx, gw_cy = 5.0, 5.35
    gw_w, gw_h = 3.2, 1.9
    box = FancyBboxPatch((gw_cx - gw_w/2, gw_cy - gw_h/2), gw_w, gw_h,
                         boxstyle="round,pad=0.02,rounding_size=0.12",
                         facecolor=COL_CH, edgecolor="black",
                         linewidth=0.7, zorder=4)
    ax.add_patch(box)
    ax.text(gw_cx, gw_cy + gw_h/2 - 0.22, "Gateway",
            ha="center", va="center", color="white",
            fontsize=8.2, fontweight="bold", zorder=5)
    ax.text(gw_cx, gw_cy + gw_h/2 - 0.46, "MqttSnGateway",
            ha="center", va="center", color="white",
            fontsize=6.5, zorder=5)
    pq_x_left = gw_cx - 1.35
    pq_y_top  = gw_cy + 0.15
    pq_w, pq_h = 2.7, 0.26
    tiers = [("CRITICAL", COL_ECG), ("HIGH", "#d35400"),
             ("MEDIUM", COL_HR), ("LOW", COL_TEMP)]
    for i, (lab, col) in enumerate(tiers):
        y = pq_y_top - i*(pq_h + 0.03)
        ax.add_patch(Rectangle((pq_x_left, y - pq_h), pq_w, pq_h,
                               facecolor=col, edgecolor="white",
                               linewidth=0.5, zorder=5))
        ax.text(pq_x_left + pq_w/2, y - pq_h/2, lab, ha="center",
                va="center", fontsize=5.8, fontweight="bold",
                color="white", zorder=6)
    ax.annotate("+1 ms  /  packet\n(header parse + QoS ACK)",
                xy=(pq_x_left + pq_w + 0.02, pq_y_top - pq_h/2),
                xytext=(gw_cx + gw_w/2 + 0.35, gw_cy + gw_h/2 + 0.6),
                fontsize=6.5, ha="left", va="bottom", color="#b03030",
                fontweight="bold",
                arrowprops=dict(arrowstyle="->", lw=0.9, color="#b03030",
                                connectionstyle="arc3,rad=-0.25"))
    for (x, y, c, _lab) in sensors:
        arrow(ax, x, y, gw_cx, gw_cy + gw_h/2 + 0.05,
              color=c, lw=0.75, mutation=7)
    pdr_marker(ax, gw_cx - gw_w/2, gw_cy + 0.45,
               "(Set 1, 3)", side="left")
    br_cx, br_cy = 5.0, 2.3
    arrow(ax, gw_cx, gw_cy - gw_h/2, br_cx, br_cy + 0.55,
          color=COL_DASHED, dashed=True, lw=1.0, mutation=9,
          shrinkA=4, shrinkB=4)
    ax.text(gw_cx + 0.18, (gw_cy - gw_h/2 + br_cy + 0.55)/2,
            "optional\n(Set 2, 3b)", fontsize=6.7, color=COL_DASHED,
            ha="left", va="center", style="italic")
    node_box(ax, br_cx, br_cy, w=2.4, h=1.1,
             label="Broker", sublabel="MqttSnBroker", color=COL_SINK)
    pdr_marker(ax, br_cx + 1.2, br_cy, "(Set 2, 3b)", side="right")
    ax.set_xlim(0, 10); ax.set_ylim(0, 10.8)
    ax.set_aspect("equal"); ax.axis("off")

def main():
    fig, (axA, axB) = plt.subplots(1, 2, figsize=(7.6, 4.4))
    draw_panel_wsn(axA)
    draw_panel_mqttsn(axB)
    fig.text(0.5, 0.055,
             "Shared substrate in both architectures:  "
             "IEEE 802.11b ad-hoc (11 Mbps, 20 dBm)   |   "
             "AODV routing   |   LogDistance propagation (exponent = 2.5)",
             ha="center", va="bottom", fontsize=7.1, style="italic",
             color="#444444")
    legend_items = [
        Line2D([0], [0], marker='o', color='w',
               markerfacecolor=COL_PDR, markeredgecolor=COL_PDR,
               markersize=6, label="PDR measurement point"),
        Line2D([0], [0], color=COL_DASHED, lw=1.1, linestyle="--",
               label="optional downstream hop (brokered configurations)"),
    ]
    fig.legend(handles=legend_items, loc="lower center",
               bbox_to_anchor=(0.5, 0.005), ncol=2, frameon=False,
               fontsize=7)
    plt.subplots_adjust(left=0.01, right=0.99, top=0.96,
                        bottom=0.11, wspace=0.03)
    fig.savefig("graphs/comparison/fig_topology.eps", format="eps",
                bbox_inches="tight", pad_inches=0.08)
    fig.savefig("graphs/comparison/fig_topology.png", format="png",
                dpi=220, bbox_inches="tight", pad_inches=0.08)
    print("[OK] fig_topology.eps + fig_topology.png saved to graphs/comparison/")
    plt.close()

if __name__ == "__main__":
    main()
