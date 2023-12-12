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
from matplotlib import colormaps
from matplotlib.widgets import TextBox

global old_poly_x, old_poly_y

# helper function
def point_to_line_dist(point, line):
    """Calculate the distance between a point and a line segment.

    To calculate the closest distance to a line segment, we first need to check
    if the point projects onto the line segment.  If it does, then we calculate
    the orthogonal distance from the point to the line.
    If the point does not project to the line segment, we calculate the 
    distance to both endpoints and take the shortest distance.

    :param point: Numpy array of form [x,y], describing the point.
    :type point: numpy.core.multiarray.ndarray
    :param line: list of endpoint arrays of form [P1, P2]
    :type line: list of numpy.core.multiarray.ndarray
    :return: The minimum distance to a point.
    :rtype: float
    """
    # unit vector
    unit_line = line[1] - line[0]
    norm_unit_line = unit_line / np.linalg.norm(unit_line)

    # compute the perpendicular distance to the theoretical infinite line
    segment_dist = (
        np.linalg.norm(np.cross(line[1] - line[0], line[0] - point)) /
        np.linalg.norm(unit_line)
    )

    diff = (
        (norm_unit_line[0] * (point[0] - line[0][0])) + 
        (norm_unit_line[1] * (point[1] - line[0][1]))
    )

    x_seg = (norm_unit_line[0] * diff) + line[0][0]
    y_seg = (norm_unit_line[1] * diff) + line[0][1]

    endpoint_dist = min(
        np.linalg.norm(line[0] - point),
        np.linalg.norm(line[1] - point)
    )

    # decide if the intersection point falls on the line segment
    lp1_x = line[0][0]  # line point 1 x
    lp1_y = line[0][1]  # line point 1 y
    lp2_x = line[1][0]  # line point 2 x
    lp2_y = line[1][1]  # line point 2 y
    is_betw_x = lp1_x <= x_seg <= lp2_x or lp2_x <= x_seg <= lp1_x
    is_betw_y = lp1_y <= y_seg <= lp2_y or lp2_y <= y_seg <= lp1_y
    if is_betw_x and is_betw_y:
        return segment_dist
    else:
        # if not, then return the minimum distance to the segment endpoints
        return endpoint_dist

# sets globals for map variable, run before everything else
def load_map(path):
    global map_image, resolution, origin_x, origin_y

    # Load the YAML file for map metadata
    yaml_file_path = 'coles_map_2.yaml'
    with open(yaml_file_path, 'r') as yaml_file:
        map_metadata = yaml.safe_load(yaml_file)

    # Load the map image
    map_image_path = map_metadata['image']
    map_image = Image.open(map_image_path)
    map_image = ImageOps.flip(map_image)

    # Extracting map details
    resolution = map_metadata['resolution']
    origin = map_metadata['origin']
    origin_x, origin_y = origin[0], origin[1]

# returns x,y of downsampled raceline
def downsample(x, y, num_samples):
    line = np.column_stack([x, y])
    length = np.cumsum(np.linalg.norm(line[1:,0:2]-line[:-1,0:2],ord=2,axis=1))[-1]
    new_line = np.zeros((num_samples, len(line[0])))
    t = np.linspace(0, 1, len(line[:,0]))
    t2 = np.linspace(0, 1, num_samples)
    for col in range(len(line[0])):
        new_line[:,col] = np.interp(t2, t, line[:,col])
        
    return new_line[:,0], new_line[:,1]

# returns x,y of smoothed raceline
def smooth_raceline(x, y, num_points=300):
    xx = np.linspace(0, 1, len(x))
    spline = scipy.interpolate.make_interp_spline(xx, np.c_[x, y], bc_type="periodic")
    x_smooth, y_smooth = spline(np.linspace(0, 1, num_points)).T
    return x_smooth, y_smooth

    # x_smooth = scipy.ndimage.gaussian_filter1d(x, sigma=sigma)
    # y_smooth = scipy.ndimage.gaussian_filter1d(y, sigma=sigma)
    #
    # raceline_data = pd.DataFrame({'x': x_smooth, 'y': y_smooth})
    #
    # avg_dist = np.mean(np.sqrt(np.sum(np.diff([x_smooth, y_smooth], axis=0)**2, axis=1)))
    #
    # first_point = raceline_data.iloc[0].values
    # last_point = raceline_data.iloc[-1].values
    #
    # distance_between_endpoints = np.linalg.norm(last_point - first_point)
    # num_points_to_add = int(np.ceil(distance_between_endpoints / avg_dist))
    #
    # new_points_x = np.linspace(last_point[0], first_point[0], num_points_to_add, endpoint=False)[1:]
    # new_points_y = np.linspace(last_point[1], first_point[1], num_points_to_add, endpoint=False)[1:]
    #
    # extended_x = np.concatenate((x_smooth, new_points_x))
    # extended_y = np.concatenate((y_smooth, new_points_y))
    #
    # return downsample(extended_x, extended_y, num_points)

# return x,y from csv
def load_raceline(path):
    data = pd.read_csv(path)
    x = (data.iloc[:,0].to_numpy() - origin_x) / resolution
    y = (data.iloc[:,1].to_numpy() - origin_y) / resolution
    if len(data.columns) >= 3:
        lookaheads = data.iloc[:,2].to_numpy()
    else:
        lookaheads = np.full(len(x), 1.5)

    return x, y, lookaheads

# saves x,y to csv
def save_raceline(x, y, lookaheads, path):
    print(len(x), len(y), len(lookaheads))
    # Exporting the DataFrame to a CSV file
    plot_x_scaled = x*resolution + origin_x
    plot_y_scaled = y*resolution + origin_y
    export_data = pd.DataFrame({'plot_x': plot_x_scaled, 'plot_y': plot_y_scaled, 'lookaheads': lookaheads})
    export_csv_path = path
    export_data.to_csv(export_csv_path, index=False, header=None)

    global old_poly_x, old_poly_y
    # self.poly.xy = np.column_stack([self.x, self.y])
    print(old_poly_x)
    new_export_data = pd.DataFrame({'plot_x': [i*resolution + origin_x for i in  old_poly_x], 'plot_y': [i*resolution + origin_y for i in  old_poly_y]})
    new_export_data.to_csv("raw_" + export_csv_path, index=False, header=None)

class PolygonInteractor(object):
    """
    A polygon editor.

    Saves final polygon in globals plot_x, plot_y

    Key-bindings

      't' toggle vertex markers on and off.  When vertex markers are on,
          you can move them, delete them

      'd' delete the vertex under point

      'i' insert a vertex at point.  You must be within epsilon of the
          line connecting two existing vertices

      'x' toggle between smoothed and unsmoothed versions

    """

    showverts = True
    epsilon = 15  # max pixel distance to count as a vertex hit
    legend = None
    lookahead_i = None

    def __init__(self, ax, x, y, lookaheads):
        if x[0] != x[-1] or y[0] != y[-1]:
            x = np.append(x, x[0])
            y = np.append(y, y[0])
            lookaheads = np.append(lookaheads, lookaheads[0])
        self.ax = ax
        self.canvas = ax.figure.canvas
        self.scatter = ax.scatter(x, y, animated=True, c=np.linspace(0, 2.5, len(x)), cmap="hsv")
        self.x = x
        self.y = y
        self.lookaheads = lookaheads
        self.line = Line2D(x, y, animated=True)
        ax.add_line(self.line)

        self._ind = None  # the active vert

        self.num_smooth_points = 300
        self.smooth = False

        self.default_lookahead = 1.5
        self.lookaheads = np.full(self.num_smooth_points, self.default_lookahead)

        global plot_x, plot_y, lookahead
        plot_x, plot_y = x, y
        lookahead = self.default_lookahead

        self.ax.set_title("Raceline")

        self.canvas.mpl_connect('draw_event', self.draw_callback)
        self.canvas.mpl_connect('button_press_event', self.button_press_callback)
        self.canvas.mpl_connect('key_press_event', self.key_press_callback)
        self.canvas.mpl_connect('button_release_event', self.button_release_callback)
        self.canvas.mpl_connect('motion_notify_event', self.motion_notify_callback)

    def draw_callback(self, event):
        self.background = self.canvas.copy_from_bbox(self.ax.bbox)
        self.ax.draw_artist(self.scatter)
        self.ax.draw_artist(self.line)

    def get_ind_under_point(self, event):
        'get the index of the vertex under point if within epsilon tolerance'

        # display coords
        xy = np.c_[self.x, self.y]
        xyt = self.scatter.get_offset_transform().transform(xy)
        xt, yt = xyt[:, 0], xyt[:, 1]
        d = np.hypot(xt - event.x, yt - event.y)
        indseq, = np.nonzero(d == d.min())
        ind = indseq[0]

        if d[ind] >= self.epsilon:
            ind = None

        return ind

    def button_press_callback(self, event):
        'whenever a mouse button is pressed'
        if not self.showverts or event.inaxes is None or event.button != 1:
            return
        self._ind = self.get_ind_under_point(event)

    def button_release_callback(self, event):
        'whenever a mouse button is released'
        if not self.showverts or event.button != 1:
            return
        self._ind = None

    def key_press_callback(self, event):
        'whenever a key is pressed'
        if not event.inaxes:
            return
        if event.key == 't':
            i = self.get_ind_under_point(event)
            if self.lookahead_i is None:
                self.lookahead_i = i
            elif i is not None:
                begin = self.lookahead_i
                end = i
                
                print(f"Changing from {begin} to {end}")

                global lookahead
                if begin < end:
                    self.lookaheads[begin:end+1] = lookahead
                else:
                    self.lookaheads[begin:] = lookahead
                    self.lookaheads[:end+1] = lookahead
                self.lookahead_i = None
        elif event.key == 'd':
            ind = self.get_ind_under_point(event)
            print("before here")
            if ind is not None:
                self.x = np.delete(self.x, ind)
                self.y = np.delete(self.y, ind)
                self.lookaheads = np.delete(self.lookaheads, ind)
        elif event.key == 'i':
            xys = self.scatter.get_offset_transform().transform(np.c_[self.x, self.y])
            p = event.x, event.y  # display coords
            for i in range(len(xys) - 1):
                s0 = xys[i]
                s1 = xys[i + 1]
                d = point_to_line_dist(p, (s0, s1))
                if d <= self.epsilon:
                    self.x = np.insert(self.x, i+1, event.xdata)
                    self.y = np.insert(self.y, i+1, event.ydata)
                    break
        elif event.key == 'x':
            global old_poly_x, old_poly_y
            if self.smooth:
                print("reverting")
                # need to revert back to unsmooth
                old_poly_x = self.x
                old_poly_y = self.y
                self.x, self.y = self.original_x, self.original_y
                self.smooth = False
            else:
                # need to smooth out 
                print("smoothing")
                old_poly_x = self.x
                old_poly_y = self.y
                self.original_x, self.original_y = self.x, self.y
                self.x, self.y = smooth_raceline(self.x, self.y, self.num_smooth_points)
                # self.scatter.set_array(self.lookaheads)
                self.smooth = True

        self.redraw()

        if self.scatter.stale:
            self.scatter.draw_idle()

    def motion_notify_callback(self, event):
        'on mouse movement'
        if not self.showverts or self._ind is None or event.inaxes is None or event.button != 1:
            return

        x, y = event.xdata, event.ydata

        self.x[self._ind] = x
        self.y[self._ind] = y
        if self._ind == 0:
            self.x[-1], self.y[-1] = x, y
        elif self._ind == len(self.x) - 1:
            self.x[0], self.y[0] = x, y

        self.redraw()

    def redraw(self):
        # save to globals
        global plot_x, plot_y, plot_l
        plot_x, plot_y = self.x, self.y
        plot_l = self.lookaheads

        self.scatter.set_offsets(np.c_[self.x, self.y])
        self.line.set_data(self.x, self.y)
        self.scatter.set_array(self.lookaheads)
        self.legend = self.ax.legend(*self.scatter.legend_elements(), title="lookahead")

        self.canvas.restore_region(self.background)
        self.ax.draw_artist(self.scatter)
        self.ax.draw_artist(self.line)
        self.ax.draw_artist(self.legend)
        self.canvas.blit(self.ax.bbox)

# plots racelines, iterable of (x,y) racelines
def draw_racelines(racelines):
    plt.cla()

    colors = colormaps.get_cmap("rainbow")(np.linspace(0, 1, len(racelines)))
    for (x, y), color in zip(racelines, colors):
        plt.scatter(x, y, color=color)

    plt.title('Overlay of Raceline Points on Map')
    plt.xlabel('Pixel X')
    plt.ylabel('Pixel Y')

    plt.show()

def draw_polygon(x, y, lookaheads):
    fig, ax = plt.subplots()
    ax.set_xlim((60, 220))
    ax.set_ylim((35, 205))
    ax.imshow(map_image, cmap='gray')

    p = PolygonInteractor(ax, x, y, lookaheads)

    def submit_lookahead(text):
        global lookahead
        lookahead = float(text)
        print(f"Changing lookahead to {lookahead}")
        p.redraw()

    axbox = fig.add_axes([0.1, 0.05, 0.8, 0.075])
    text_box = TextBox(axbox, "Lookahead distance")
    text_box.on_submit(submit_lookahead)
    text_box.set_val("1.5")

    plt.show()

    return plot_x, plot_y, plot_l

load_map("coles_map_2.yaml")
# x, y, lookaheads = load_raceline("justins_rough.csv")
x, y, lookaheads = load_raceline(input("enter starting raceline (include.csv): "))

# x, y = downsample(x, y, 10)
x, y, lookaheads = draw_polygon(x, y, lookaheads)

print(len(lookaheads))
save_raceline(x, y, lookaheads, input("Enter output filename: "))
