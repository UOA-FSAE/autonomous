import numpy as np

n = 8  # Example value, you can change it to any desired size

# Create a 3x3 identity matrix
identity_3x3 = np.eye(3)
vector = np.array([[1],[0],[0]])

print(np.matmul(identity_3x3, vector))
