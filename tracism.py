import pandas as pd
import matplotlib.pyplot as plt
import sys

# Load the CSV file
file_path = sys.argv[0]  # Replace with your file path
data = pd.read_csv(file_path, header=None)
data.columns = ['Column1', 'Column2', 'Column3']

# Replace NaN values with 5
data['Column3'].fillna(5, inplace=True)

# Replace values of 0 with 5 in Column 3
data['Column3'].replace(0, 5, inplace=True)

# Calculate the time for each row, assuming each row is 0.1 seconds apart
time = [0.1 * i for i in range(len(data))]

# Setting the dark background style
plt.style.use('dark_background')

# Plotting the graph
plt.figure(figsize=(12, 6))
plt.plot(time, data['Column3'], color='lime', label='Column 3')
plt.xlabel('Time (seconds)')
plt.ylabel('Value of Column 3')
plt.title('Plot of Column 3 vs Time (Dark Background, Neon Green Line)')
plt.legend()
plt.show()
