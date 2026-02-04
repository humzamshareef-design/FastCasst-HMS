import numpy as np
import matplotlib.pyplot as plt
import glob
import os

files = sorted(glob.glob(os.path.join("logs", "*.txt")))
fname = files[-1]

data = np.loadtxt(fname, delimiter=",")

t   = data[:,0]
x   = data[:,1]
y   = data[:,2]
u   = data[:,3]
psi = data[:,4]
thr = data[:,5]
ste = data[:,6]
mode= data[:,7]

plt.figure(figsize=(12, 8))

plt.subplot(2,2,1)
plt.plot(x, y)
plt.xlabel("x (m)")
plt.ylabel("y (m)")
plt.title("Trajectory")
plt.axis("equal")
plt.grid()

plt.subplot(2,2,2)
plt.plot(t, u)
plt.xlabel("Time (s)")
plt.ylabel("Speed (m/s)")
plt.title("Speed")
plt.grid()

plt.subplot(2,2,3)
plt.plot(t, psi)
plt.xlabel("Time (s)")
plt.ylabel("Heading (deg)")
plt.title("Heading")
plt.grid()

plt.subplot(2,2,4)
plt.plot(t, x, label="x")
plt.plot(t, y, label="y")
plt.xlabel("Time (s)")
plt.ylabel("Position (m)")
plt.title("Position vs Time")
plt.legend()
plt.grid()

plt.tight_layout()
plt.show()