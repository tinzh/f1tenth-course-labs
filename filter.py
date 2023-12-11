import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from PIL import Image
import yaml
import numpy as np
from PIL import ImageOps
import scipy.interpolate
import scipy.ndimage

# Load the map image
map_image_path = 'coles_map_2.pgm'
map_image = Image.open(map_image_path)
map_image = ImageOps.flip(map_image)

# Load the YAML file for map metadata
yaml_file_path = 'coles_map_2.yaml'
with open(yaml_file_path, 'r') as yaml_file:
    map_metadata = yaml.safe_load(yaml_file)

# Extracting map details
resolution = map_metadata['resolution']
origin = map_metadata['origin']
origin_x, origin_y = origin[0], origin[1]

def downsample_dist(line, dist):
    length = np.cumsum(np.linalg.norm(line[1:,0:2]-line[:-1,0:2],ord=2,axis=1))[-1]
    num_samples = int(length/dist)
    new_line = np.zeros((num_samples, len(line[0])))
    t = np.linspace(0, 1, len(line[:,0]))
    t2 = np.linspace(0, 1, num_samples)
    for col in range(len(line[0])):
        new_line[:,col] = np.interp(t2, t, line[:,col])
        
    return new_line

# coles = pd.read_csv("coles_filtered_raceline.csv")
# justins = pd.read_csv("justins_flitered_raceline.csv")
# coles.columns = ["x", "y"]
# justins.columns = ["x", "y"]
# avg = pd.read_csv("cleaned_averaged_raceline.csv")
# avg.columns = ["x", "y"]

data = pd.read_csv("justins_raceline.csv")
data.columns = ["X", "Y", "Z", "W"]

line = np.column_stack([np.asarray(data['X']), np.asarray(data['Y'])])

new_line = line

sig = 3
x_smooth = scipy.ndimage.gaussian_filter1d(new_line[:,0], sigma=sig)
y_smooth = scipy.ndimage.gaussian_filter1d(new_line[:,1], sigma=sig)

x_smooth = [(i-origin_x)/resolution for i in x_smooth]
y_smooth = [(i-origin_y)/resolution for i in y_smooth]

sig = 8
x_smooth_new = scipy.ndimage.gaussian_filter1d(new_line[:,0], sigma=sig)
y_smooth_new = scipy.ndimage.gaussian_filter1d(new_line[:,1], sigma=sig)

x_smooth_new = [(i-origin_x)/resolution for i in x_smooth_new][5:-5]
y_smooth_new = [(i-origin_y)/resolution for i in y_smooth_new][5:-5]


# Create DataFrame from points_x and points_y
raceline_data = pd.DataFrame({'x': x_smooth, 'y': y_smooth})

# print(raceline_data)

# Function to calculate the average distance between points in a line
def average_distance(points):
    distances = np.sqrt(np.sum(np.diff(points, axis=0)**2, axis=1))
    return np.mean(distances)

# Calculate the average distance between points in the raceline
avg_dist = average_distance(raceline_data.values)

# Get the first and last point of the raceline
first_point = raceline_data.iloc[0].values
last_point = raceline_data.iloc[-1].values

# Calculate the number of points needed to interpolate based on the average distance
distance_between_endpoints = np.linalg.norm(last_point - first_point)
num_points_to_add = int(np.ceil(distance_between_endpoints / avg_dist))

# Generate the new points
new_points_x = np.linspace(last_point[0], first_point[0], num_points_to_add, endpoint=False)[1:]
new_points_y = np.linspace(last_point[1], first_point[1], num_points_to_add, endpoint=False)[1:]

# Create DataFrame for the new points
new_points = pd.DataFrame({'x': new_points_x, 'y': new_points_y})

# Append the new points to the end of the raceline data
extended_raceline_data = pd.concat([raceline_data, new_points], ignore_index=True)

line = np.column_stack([np.asarray(extended_raceline_data['x']), np.asarray(extended_raceline_data['y'])])
new_line = downsample_dist(line, 0.1/resolution)
plot_x = new_line[:,0]
plot_y = new_line[:,1]
plot_x_scaled = plot_x*resolution + origin_x
plot_y_scaled = plot_y*resolution + origin_y

# Plotting the points on the map
plt.figure(figsize=(12, 12))
plt.imshow(map_image, cmap='gray')
# plt.scatter(extended_raceline_data['x'], extended_raceline_data['y'], c='red', s=10)  # red points for raceline
# plt.scatter(plot_x, plot_y, c='green', s=10)  # red points for raceline
# plt.scatter(coles["x"], coles["y"], c='red', s=10)  # red points for raceline
# plt.scatter(justins["x"], justins["y"], c='blue', s=10)  # red points for raceline
plt.scatter(plot_x, plot_y, c='orange', s=10)  # red points for raceline
# plt.scatter((data['X']-origin_x)/resolution, (data['Y']-origin_y)/resolution, c='blue', s=10)  # red points for raceline
plt.gca().invert_yaxis()  # Inverting Y-axis to match image coordinate system
plt.title('Overlay of Raceline Points on Map')
plt.xlabel('Pixel X')
plt.ylabel('Pixel Y')
plt.show()

export_data = pd.DataFrame({'plot_x': plot_x_scaled, 'plot_y': plot_y_scaled})

# Exporting the DataFrame to a CSV file
export_csv_path = 'justins_scaled_raceline.csv'
export_data.to_csv(export_csv_path, index=False, header=None)
