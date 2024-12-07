import matplotlib.pyplot as plt
import pandas as pd
from io import StringIO

# Given data
data = """
positions_X,positions_y
0.7415,3.1583
0.7383,3.0585
0.7256,2.9594
0.7068,2.8612
0.6829,2.7641
0.6583,2.6687
0.6441,2.5698
0.6454,2.4701
0.6632,2.3722
0.6954,2.2776
0.7427,2.1897
0.7962,2.1061
0.8394,2.0161
0.8688,1.9207
0.8829,1.8224
0.8787,1.7227
0.8585,1.6251
0.8244,1.5313
0.7798,1.4421
0.7286,1.3561
0.6733,1.2728
0.6210,1.1878
0.5808,1.0964
0.5606,0.9984
0.5533,0.8995
0.5618,0.8002
0.5908,0.7046
0.6357,0.6163
0.6908,0.5328
0.7640,0.4665
0.8442,0.4068
0.9324,0.3618
1.0268,0.3293
1.1246,0.3083
1.2192,0.2810
1.3084,0.2366
1.3900,0.1797
1.4608,0.1092
1.5158,0.0258
1.5551,-0.0658
1.5813,-0.1620
1.5893,-0.2615
1.5783,-0.3608
1.5554,-0.4572
1.5431,-0.5563
1.5468,-0.6560
1.5671,-0.7536
1.6003,-0.8477
1.6434,-0.9379
1.6891,-1.0268
1.7372,-1.1145
1.7870,-1.2012
1.8425,-1.2843
1.9004,-1.3656


"""

# Reading data into a DataFrame
df = pd.read_csv(StringIO(data))

# Calculate the reference point (midpoint of positions_X and positions_y)
ref_x = df['positions_X'].mean()
ref_y = df['positions_y'].mean()

# Shift the positions relative to the reference point
df['positions_X_relative'] = df['positions_X'] - ref_x
df['positions_y_relative'] = df['positions_y'] - ref_y

# Plotting with data converted to NumPy arrays
plt.figure(figsize=(10, 6))
plt.plot(df['positions_X_relative'].to_numpy(), df['positions_y_relative'].to_numpy(), label='Position (X, Y) relative to Reference Point', marker='o')
plt.scatter(0, 0, color='red', label='Reference Point (0, 0)', marker='x')  # Mark the reference point
plt.xlabel('Position X (relative to Reference Point)')
plt.ylabel('Position Y (relative to Reference Point)')
plt.title('Visualization of Positions Relative to Reference Point (Midpoint)')
plt.legend()
plt.grid(True)
plt.show()
