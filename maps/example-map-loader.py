import numpy as np

# Step 1: Read the map from a text file
with open('apartment-40x40.txt', 'r') as file:
    lines = file.readlines()

# Step 2: Convert the map into a NumPy array
map_array = np.array([list(line.strip()) for line in lines], dtype=int)

# Step 3: Use map for e.g. navigation
print(map_array)
