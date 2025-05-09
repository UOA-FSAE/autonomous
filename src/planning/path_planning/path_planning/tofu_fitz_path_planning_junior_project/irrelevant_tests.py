import numpy as np
import matplotlib.pyplot as plt

x = []
y = []
yellow = []
blue = []

# Open and read the file
with open('berlin_2018.txt', 'r') as file:
    for line in file:
        parts = line.strip().split(',')
        if len(parts) == 4:
            x.append(float(parts[0]))
            y.append(float(parts[1]))
            yellow.append(float(parts[2]))
            blue.append(float(parts[3]))

# Convert to NumPy array for convenience
points = np.column_stack((x, y))

# Plot x vs y
plt.figure(figsize=(8, 6))
plt.plot(x, y, 'k-o', label='Center Line')
plt.xlabel('x (m)')
plt.ylabel('y (m)')
plt.title('Track Center Line')
plt.grid(True)
plt.axis('equal')
plt.legend()
plt.show()
