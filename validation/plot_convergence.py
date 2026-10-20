"""Plots validation/out/csharp/convergence_study.csv -> validation/plots/convergence_study.png."""
import csv, os, sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

here = os.path.dirname(os.path.abspath(__file__))
src = os.path.join(here, "out", "csharp", "convergence_study.csv")
dst = os.path.join(here, "plots", "convergence_study.png")
rows = list(csv.DictReader(open(src)))
cases = ["r1_launch", "wot_3rd", "coast_4th"]
style = {"SemiImplicitEuler": ("#2a78d6", "o", "-", "Euler (model.m scheme)"),
         "RungeKutta4": ("#eb6834", "s", "--", "RK4")}
fig, axes = plt.subplots(1, 3, figsize=(13, 4.2), sharey=True)
fig.patch.set_facecolor("#fcfcfb")
for ax, c in zip(axes, cases):
    ax.set_facecolor("#fcfcfb")
    for integ, (col, mk, ls, lab) in style.items():
        r = sorted((float(x["dt"]), max(float(x["err_d_m"]), 1e-16)) for x in rows if x["case"] == c and x["integrator"] == integ)
        ax.loglog([a for a, _ in r], [b for _, b in r], ls, color=col, marker=mk, markersize=7, linewidth=2, label=lab)
    ax.axvline(0.005, color="#52514e", linewidth=1, linestyle=":")
    ax.set_title(c, color="#0b0b0b")
    ax.set_xlabel("dt [s]", color="#52514e")
    ax.grid(True, which="major", color="#e4e3df", linewidth=0.6)
    for s in ax.spines.values(): s.set_color("#c3c2b7")
    ax.tick_params(colors="#52514e")
axes[0].set_ylabel("|d(10 s) - d_ref| [m]  (ref: RK4, dt = 1e-5 s)", color="#52514e")
axes[0].legend(frameon=False)
fig.suptitle("Interactive layer: distance error at t = 10 s vs timestep (dotted line: game/model.m dt = 0.005 s)", color="#0b0b0b")
fig.tight_layout()
fig.savefig(dst, dpi=110, facecolor=fig.get_facecolor())
print("wrote", dst)
