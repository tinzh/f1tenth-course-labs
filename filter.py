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
    return (data.iloc[:,0] - origin_x) / resolution, (data.iloc[:,1] - origin_y) / resolution;

# saves x,y to csv
def save_raceline(x, y, path):
    # Exporting the DataFrame to a CSV file
    plot_x_scaled = x*resolution + origin_x
    plot_y_scaled = y*resolution + origin_y
    export_data = pd.DataFrame({'plot_x': plot_x_scaled, 'plot_y': plot_y_scaled})
    export_csv_path = path
    export_data.to_csv(export_csv_path, index=False, header=None)

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

        self.num_smooth_points = 100
        self.smooth = False
        global plot_x, plot_y
        plot_x, plot_y = x, y

        self.ax.set_title("Raceline")

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
                d = point_to_line_dist(p, (s0, s1))
                if d <= self.epsilon:
                    self.poly.xy = np.insert(
                        self.poly.xy, i+1,
                        [event.xdata, event.ydata],
                        axis=0)
                    self.line.set_data(zip(*self.poly.xy))
                    break
        elif event.key == 'x':
            print("here")
            if self.smooth:
                print("reverting")
                # need to revert back to unsmooth
                self.poly.xy = np.column_stack([self.x, self.y])
                global plot_x, plot_y
                plot_x, plot_y = self.x, self.y
                self.smooth = False
            else:
                # need to smooth out 
                print("smoothing")
                self.x = self.poly.xy[:,0]
                self.y = self.poly.xy[:,1]
                
                x_smooth, y_smooth = smooth_raceline(self.x, self.y)
                self.poly.xy = np.column_stack([x_smooth, y_smooth])
                plot_x, plot_y = x_smooth, y_smooth
                self.smooth = True

            self.redraw()

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

        self.redraw()

    def redraw(self):
        self.line.set_data(zip(*self.poly.xy))
        self.canvas.restore_region(self.background)
        self.ax.draw_artist(self.poly)
        self.ax.draw_artist(self.line)
        self.canvas.blit(self.ax.bbox)

        global plot_x, plot_y
        plot_x = self.poly.xy[:,0]
        plot_y = self.poly.xy[:,1]

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

def draw_polygon(x, y):
    poly = Polygon(np.column_stack([x, y]), animated=True, fill=False)
    fig, ax = plt.subplots()
    ax.add_patch(poly)
    p = PolygonInteractor(ax, poly)
    ax.set_xlim((60, 220))
    ax.set_ylim((35, 205))
    ax.imshow(map_image, cmap='gray')
    plt.show()
    return plot_x, plot_y

load_map("coles_map_2.yaml")
x, y = load_raceline("justins_rough.csv")
# x, y = downsample(x, y, 10)
x, y = draw_polygon(x, y)
save_raceline(x, y, input("Enter output filename: "))
