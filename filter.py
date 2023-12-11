import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import scipy.interpolate
import scipy.ndimage

data = pd.read_csv("raceline.csv")
data.columns = ["X", "Y", "Z", "W"]
data['X'] = data['X']
data['Y'] = data['Y']

sigma = 1

def create_looped_line(df):
    df = df.append(df.iloc[0], ignore_index=True)

    num_points = 100
    t_new = np.linspace(0, 1, num_points)
    t_old = np.linspace(0, 1, len(df))
    f_x = scipy.interpolate.interp1d(t_old, df['X'], kind='cubic')
    f_y = scipy.interpolate.interp1d(t_old, df['Y'], kind='cubic')
    
    x_smooth = scipy.ndimage.gaussian_filter1d(f_x(t_new), sigma=sigma)
    y_smooth = scipy.ndimage.gaussian_filter1d(f_y(t_new), sigma=sigma)
    
    return x_smooth, y_smooth

x_smooth, y_smooth = create_looped_line(data[['X', 'Y']])

plt.scatter(data['X'], data['Y'], color='red', label='Original Data')
plt.plot(x_smooth, y_smooth, color='blue', label='Smoothed Line')
plt.legend()
plt.show()