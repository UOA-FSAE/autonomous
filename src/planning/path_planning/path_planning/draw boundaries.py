import matplotlib.pyplot as plt

with open('/home/fsae/Autonomous_Repos/autonomous_nightly/src/planning/path_planning/path_planning/bound_coods', 'r') as fh:
    xl = fh.readline()
    yl = fh.readline()
    xr = fh.readline()
    yr = fh.readline()
with open('/home/fsae/Autonomous_Repos/autonomous_nightly/src/planning/path_planning/path_planning/bound_coods2', 'r') as fh:
    xl2 = fh.readline()
    yl2 = fh.readline()
    xr2 = fh.readline()
    yr2 = fh.readline()
yl = [float(P) for i,P in enumerate(yl.split(" ")) if i!=len(yl.split(" "))-1]
xl = [float(P) for i,P in enumerate(xl.split(" ")) if i!=len(xl.split(" "))-1]
xr = [float(P) for i,P in enumerate(xr.split(" ")) if i!=len(xr.split(" "))-1]
yr = [float(P) for i,P in enumerate(yr.split(" ")) if i!=len(yr.split(" "))-1]

yl2 = [float(P) for i,P in enumerate(yl2.split(" ")) if i!=len(yl2.split(" "))-1]
xl2 = [float(P) for i,P in enumerate(xl2.split(" ")) if i!=len(xl2.split(" "))-1]
xr2 = [float(P) for i,P in enumerate(xr2.split(" ")) if i!=len(xr2.split(" "))-1]
yr2 = [float(P) for i,P in enumerate(yr2.split(" ")) if i!=len(yr2.split(" "))-1]

plt.plot(xl,yl,'ob',label='left boundary')
plt.plot(xr,yr,'oy',label='right boundary')

plt.plot(xl2,yl2,'or',label='left boundary original')
plt.plot(xr2,yr2,'og',label='right boundary original')

plt.legend()
plt.show()