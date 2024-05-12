import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import CubicSpline
from matplotlib.patches import Rectangle, Circle


df = pd.read_csv("data/mobility-data-4.csv")

n1laps = 6
x1lap = [1.0,0.4,0.4,1.2,1.6,1.6]
y1lap = [0.4,1.0,2.0,2.5,2.0,1.0]

nlaps = 5
nc = n1laps*nlaps+1
xc = np.zeros(nc)
yc = np.zeros(nc)
for i in range(nc):
    xc[i] = x1lap[i%6]
    yc[i] = y1lap[i%6]
qc = np.arange(0,nc)
# q = np.linspace(3*nlaps, nc-2*nlaps, 5000)
q = np.linspace(0, nc-1, 5000)
xref = CubicSpline(qc, xc)(q)
yref = CubicSpline(qc, yc)(q)

dist = np.sum(np.sqrt(np.diff(df["x"])**2 + np.diff(df["y"])**2))
print("Total traveled distance: %.3f" % dist)


fig, ax = plt.subplots(figsize=(10/1.5,7/1.5))
# ax.grid()
ax.set_title("Robot path")
ax.add_patch(Rectangle((0, 0), 3.0, 2.0, color='k', fill=False, linewidth=2))
mask = np.logical_and(df["time"]/1e6 <= 80, df["time"]/1e6 >= 20)
mask = np.ones_like(mask, dtype=bool)
ax.plot(df["y"][mask], df["x"][mask], label="Actual trajectory")
ax.plot(yref, xref, ':', label="Reference trajectory")
ax.plot(y1lap, x1lap, "o", label="Checkpoints")
ax.axis("equal")

ax.set_xlabel("y [m]")
ax.set_ylabel("x [m]")
ax.set_ylim(ax.get_ylim()[::-1])
ax.legend()
fig.tight_layout()
plt.savefig("data/path-following-test.png")
plt.savefig("data/path-following-test.pdfx²")