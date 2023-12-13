import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import sys

def plot_obstacle_data(csv_file):
    # Load the CSV file with the correct delimiter and without a header row
    data = pd.read_csv(csv_file)

    # Extract x and y positions (assuming the first two columns are x and y)
    x = data.iloc[:, 0]
    y = data.iloc[:, 1]

    # Create a color gradient
    colors = np.linspace(0, 1, len(x))

    # Plot each point with a slightly different color
    plt.figure(figsize=(10, 6))
    for i in range(len(x)):
        plt.scatter(x[i], y[i], color=plt.cm.viridis(colors[i]))

    # Set plot labels and title
    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.title('Obstacle Data Plot')

    # Show the plot
    plt.show()

# Replace 'your_data.csv' with the path to your CSV file
plot_obstacle_data(sys.argv[0])
