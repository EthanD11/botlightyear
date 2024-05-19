import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import CubicSpline
from matplotlib.patches import Rectangle, Circle


df = pd.read_csv("data/new-test-pf/mobility-data-2.csv")

# n1laps = 
x1lap = [1.0, 0.5,  1.0, 1.5,  1.0, 0.5, 1.0, 1.5]
y1lap = [0.4, 0.8, 1.28, 1.8, 2.2, 1.8, 1.28, 0.8] 

# x1lap = [1.0, 0.5, 1.0, 1.5,  1.0, 0.5, 1.0, 1.5]
# y1lap = [0.4, 0.9, 1.4, 1.9, 2.25, 1.9, 1.4, 0.9]
n1laps = len(x1lap)

nlaps = 3
nc = n1laps*nlaps+1
xc = np.zeros(nc)
yc = np.zeros(nc)
for i in range(nc):
    xc[i] = x1lap[i%n1laps]
    yc[i] = y1lap[i%n1laps]
qc = np.arange(0,nc)
# q = np.linspace(3*nlaps, nc-2*nlaps, 5000)
q = np.linspace(n1laps, nc-n1laps-1, 5000)
xref = CubicSpline(qc, xc)(q)
yref = CubicSpline(qc, yc)(q)

dist = np.sum(np.sqrt(np.diff(df["x"])**2 + np.diff(df["y"])**2))
print("Total traveled distance: %.3f" % dist)

size_full = 1.8
size_start = 2.5
size_end = 2.5
col_pos_ctrl = "tab:blue"
col_path_flw = "tab:blue"
col_ref = "tab:orange"
col_check = "tab:green"

# FULL PLOT
mask_pf = df["mode"] == "ModePathFollowing"
mask_pc = df["mode"] == "ModePositionControl"

fig, ax = plt.subplots(figsize=(10/size_full,9/size_full))
ax.set_title("Entire trajectory")
ax.plot(df["y"][mask_pf], df["x"][mask_pf], '-', label="Path following", color=col_path_flw)
ax.plot(df["y"][mask_pc], df["x"][mask_pc], '--', label="Position control", color=col_pos_ctrl)
ax.plot(yref, xref, '-', label="Reference trajectory", color=col_ref)
ax.plot(y1lap, x1lap, "o", label="Checkpoints", color=col_check)
ax.axis("equal")

ax.set_xlabel("y [m]")
ax.set_ylabel("x [m]")
ax.set_ylim(ax.get_ylim()[::-1])
ax.legend(loc="lower right")

fig.tight_layout()
plt.savefig("data/path-following-test.png")
plt.savefig("data/path-following-test.pdf")


# START PLOT
mask = df["time"] <= 3.9
q = np.linspace(0, 2, 5000)
xref = CubicSpline(qc, xc)(q)
yref = CubicSpline(qc, yc)(q)


fig, ax = plt.subplots(figsize=(10/size_start,7/size_start))
# ax.grid()
ax.set_title("Zoom on the transient at the start")
ax.plot(df["y"][mask], df["x"][mask], '-', label="Robot trajectory", color=col_path_flw)
ax.plot(yref, xref, '-', label="Reference trajectory", color=col_ref)
ax.plot(y1lap[:3], x1lap[:3], "o", label="Checkpoints", color=col_check)
ax.axis("equal")

ax.set_xlabel("y [m]")
ax.set_ylabel("x [m]")
ax.set_ylim(ax.get_ylim()[::-1])
ax.legend()
fig.tight_layout()
plt.savefig("data/path-following-test-start.png")
plt.savefig("data/path-following-test-start.pdf")



# END PLOT
mask = df["mode"] == "ModePositionControl"

fig, ax = plt.subplots(figsize=(10/size_start,7/size_start))
# ax.grid()
ax.set_title("Zoom on the position control")
ax.plot(df["y"][mask], df["x"][mask], '-', label="Robot trajectory")
# ax.plot(yref, xref, '--', label="Reference trajectory")
ax.plot(y1lap[:1], x1lap[:1], "o", color="tab:green", label="Reference point")
ax.axis("equal")

ax.set_xlabel("y [m]")
ax.set_ylabel("x [m]")
ax.set_ylim(ax.get_ylim()[::-1])
ax.legend()
fig.tight_layout()
plt.savefig("data/path-following-test-end.png")
plt.savefig("data/path-following-test-end.pdf")