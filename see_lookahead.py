import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

def plot_csv_with_lookahead(file_path):
    # Load the CSV file
    data = pd.read_csv(file_path)

    # Assuming the file has no headers and the columns are ['x', 'y', 'lookahead']
    data.columns = ['x', 'y', 'lookahead']

    # Plotting
    plt.figure(figsize=(10, 6))
    sns.scatterplot(x='x', y='y', hue='lookahead', palette='viridis', data=data)
    plt.title('Scatter Plot with Different Lookahead Distances')
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.legend(title='Lookahead Distance')
    plt.grid(True)
    plt.show()

# Replace 'path_to_your_csv_file.csv' with the actual path to your CSV file

file_path = '118.csv'
plot_csv_with_lookahead(file_path)
