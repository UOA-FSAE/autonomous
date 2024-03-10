import matplotlib.pyplot as plt

with open('/home/tanish/autonomous/src/planning/path_planning/path_planning/bound_coods', 'r') as fh:
    xl = fh.readline()
    yl = fh.readline()
    xr = fh.readline()
    yr = fh.readline()
yl = [float(P) for i,P in enumerate(yl.split(" ")) if i!=len(yl.split(" "))-1]
xl = [float(P) for i,P in enumerate(xl.split(" ")) if i!=len(xl.split(" "))-1]
xr = [float(P) for i,P in enumerate(xr.split(" ")) if i!=len(xr.split(" "))-1]
yr = [float(P) for i,P in enumerate(yr.split(" ")) if i!=len(yr.split(" "))-1]

plt.plot(xl,yl,'ob',label='left boundary')
plt.plot(xr,yr,'oy',label='right boundary')

plt.legend()
plt.show()
plt.savefig("/home/tanish/autonomous/src/planning/path_planning/path_planning/boundaries_drawn.png")