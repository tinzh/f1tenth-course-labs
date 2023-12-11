import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from PIL import Image
import yaml
import numpy as np
from PIL import ImageOps
import scipy.interpolate
import scipy.ndimage

from matplotlib.lines import Line2D
from matplotlib.artist import Artist
from matplotlib.patches import Polygon
# from matplotlib.mlab import dist_point_to_segment
class PolygonInteractor(object):
    """
    A polygon editor.

    Key-bindings

      't' toggle vertex markers on and off.  When vertex markers are on,
          you can move them, delete them

      'd' delete the vertex under point

      'i' insert a vertex at point.  You must be within epsilon of the
          line connecting two existing vertices

    """

    showverts = True
    epsilon = 5  # max pixel distance to count as a vertex hit

    def __init__(self, ax, poly):
        if poly.figure is None:
            raise RuntimeError('You must first add the polygon to a figure '
                               'or canvas before defining the interactor')
        self.ax = ax
        canvas = poly.figure.canvas
        self.poly = poly

        x, y = zip(*self.poly.xy)
        self.line = Line2D(x, y,
                           marker='o', markerfacecolor='r',
                           animated=True)
        self.ax.add_line(self.line)

        self.cid = self.poly.add_callback(self.poly_changed)
        self._ind = None  # the active vert

        canvas.mpl_connect('draw_event', self.draw_callback)
        canvas.mpl_connect('button_press_event', self.button_press_callback)
        canvas.mpl_connect('key_press_event', self.key_press_callback)
        canvas.mpl_connect('button_release_event', self.button_release_callback)
        canvas.mpl_connect('motion_notify_event', self.motion_notify_callback)
        self.canvas = canvas

    def draw_callback(self, event):
        self.background = self.canvas.copy_from_bbox(self.ax.bbox)
        self.ax.draw_artist(self.poly)
        self.ax.draw_artist(self.line)
        # do not need to blit here, this will fire before the screen is
        # updated

    def poly_changed(self, poly):
        'this method is called whenever the polygon object is called'
        # only copy the artist props to the line (except visibility)
        vis = self.line.get_visible()
        Artist.update_from(self.line, poly)
        self.line.set_visible(vis)  # don't use the poly visibility state


    def get_ind_under_point(self, event):
        'get the index of the vertex under point if within epsilon tolerance'

        # display coords
        xy = np.asarray(self.poly.xy)
        xyt = self.poly.get_transform().transform(xy)
        xt, yt = xyt[:, 0], xyt[:, 1]
        d = np.hypot(xt - event.x, yt - event.y)
        indseq, = np.nonzero(d == d.min())
        ind = indseq[0]

        if d[ind] >= self.epsilon:
            ind = None

        return ind

    def button_press_callback(self, event):
        'whenever a mouse button is pressed'
        if not self.showverts:
            return
        if event.inaxes is None:
            return
        if event.button != 1:
            return
        self._ind = self.get_ind_under_point(event)

    def button_release_callback(self, event):
        'whenever a mouse button is released'
        if not self.showverts:
            return
        if event.button != 1:
            return
        self._ind = None

    def key_press_callback(self, event):
        'whenever a key is pressed'
        if not event.inaxes:
            return
        if event.key == 't':
            self.showverts = not self.showverts
            self.line.set_visible(self.showverts)
            if not self.showverts:
                self._ind = None
        elif event.key == 'd':
            ind = self.get_ind_under_point(event)
            if ind is not None:
                self.poly.xy = np.delete(self.poly.xy,
                                         ind, axis=0)
                self.line.set_data(zip(*self.poly.xy))
        elif event.key == 'i':
            xys = self.poly.get_transform().transform(self.poly.xy)
            p = event.x, event.y  # display coords
            for i in range(len(xys) - 1):
                s0 = xys[i]
                s1 = xys[i + 1]
                d = matplotlib.mlab.dist_point_to_segment(p, s0, s1)
                if d <= self.epsilon:
                    self.poly.xy = np.insert(
                        self.poly.xy, i+1,
                        [event.xdata, event.ydata],
                        axis=0)
                    self.line.set_data(zip(*self.poly.xy))
                    break
        if self.line.stale:
            self.canvas.draw_idle()

    def motion_notify_callback(self, event):
        'on mouse movement'
        if not self.showverts:
            return
        if self._ind is None:
            return
        if event.inaxes is None:
            return
        if event.button != 1:
            return
        x, y = event.xdata, event.ydata

        self.poly.xy[self._ind] = x, y
        if self._ind == 0:
            self.poly.xy[-1] = x, y
        elif self._ind == len(self.poly.xy) - 1:
            self.poly.xy[0] = x, y
        self.line.set_data(zip(*self.poly.xy))

        self.canvas.restore_region(self.background)
        self.ax.draw_artist(self.poly)
        self.ax.draw_artist(self.line)
        self.canvas.blit(self.ax.bbox)

        global plot_x, plot_y
        plot_x = self.poly.xy[:,0]
        plot_y = self.poly.xy[:,1]

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

# # Plotting the points on the map
# plt.figure(figsize=(12, 12))
# plt.imshow(map_image, cmap='gray')
# # plt.scatter(extended_raceline_data['x'], extended_raceline_data['y'], c='red', s=10)  # red points for raceline
# # plt.scatter(plot_x, plot_y, c='green', s=10)  # red points for raceline
# # plt.scatter(coles["x"], coles["y"], c='red', s=10)  # red points for raceline
# # plt.scatter(justins["x"], justins["y"], c='blue', s=10)  # red points for raceline
# plt.scatter(plot_x, plot_y, c='orange', s=10)  # red points for raceline
# # plt.scatter((data['X']-origin_x)/resolution, (data['Y']-origin_y)/resolution, c='blue', s=10)  # red points for raceline
# plt.gca().invert_yaxis()  # Inverting Y-axis to match image coordinate system
# plt.title('Overlay of Raceline Points on Map')
# plt.xlabel('Pixel X')
# plt.ylabel('Pixel Y')
# plt.show()

poly = Polygon(np.column_stack([plot_x, plot_y]), animated=True, fill=False)
fig, ax = plt.subplots()
ax.add_patch(poly)
p = PolygonInteractor(ax, poly)
ax.set_xlim((60, 220))
ax.set_ylim((35, 205))
ax.imshow(map_image, cmap='gray')
# ax.invert_yaxis()
plt.show()

plot_x_scaled = plot_x*resolution + origin_x
plot_y_scaled = plot_y*resolution + origin_y
export_data = pd.DataFrame({'plot_x': plot_x_scaled, 'plot_y': plot_y_scaled})

# Exporting the DataFrame to a CSV file
export_csv_path = 'justins_scaled_raceline.csv'
export_data.to_csv(export_csv_path, index=False, header=None)
