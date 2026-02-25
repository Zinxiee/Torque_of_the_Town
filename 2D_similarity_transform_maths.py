import numpy as np

# 1. ENTER YOUR WORLD COORDINATES HERE (mm)
# Format: [X, Y]
A1 = [-243.0, -82.0]  # BL Corner 1 World
A2 = [-55.0, -86.0]  # BR Corner 2 World
A3 = [-53.0, 82.0] # TR Corner 3 World
A4 = [-236.0, 82.0] # TL Corner 4 World

# 2. ENTER YOUR CAMERA COORDINATES HERE (Pixels)
# Format: [X, Y]
B1 = [54.0, 77.0]   # BL Corner 1 Pixels
B2 = [226.0, 73.0]  # BR Corner 2 Pixels
B3 = [225.0, 235.0] # TR Corner 3 Pixels
B4 = [57.0, 228.0]  # TL Corner 4 Pixels

# --- THE MATH (Do not change) ---
A = np.array([
    [A1[0]], [A1[1]], 
    [A2[0]], [A2[1]], 
    [A3[0]], [A3[1]], 
    [A4[0]], [A4[1]]
])

B = np.array([
    [B1[0], -B1[1], 1, 0], [B1[1],  B1[0], 0, 1],
    [B2[0], -B2[1], 1, 0], [B2[1],  B2[0], 0, 1],
    [B3[0], -B3[1], 1, 0], [B3[1],  B3[0], 0, 1],
    [B4[0], -B4[1], 1, 0], [B4[1],  B4[0], 0, 1]
])

# Solve: inv(B' * B) * B' * A
B_T = B.transpose()
params = np.linalg.inv(B_T.dot(B)).dot(B_T).dot(A)

print("COPY THESE 4 VALUES INTO YOUR ESP32 CODE:")
print(f"float p1 = {params[0][0]:.6f}; // k*cos(theta)")
print(f"float p2 = {params[1][0]:.6f}; // k*sin(theta)")
print(f"float tx = {params[2][0]:.6f}; // Translation X")
print(f"float ty = {params[3][0]:.6f}; // Translation Y")
