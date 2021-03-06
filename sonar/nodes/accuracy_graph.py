#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.mlab import griddata
import math

def pol2cart(rho, phi):
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return(x, y) 

def plot_grid_graph(x_list,y_list,z,z_list,typemeasure):

    hydro0_xyz = [0,      0,     0]
    hydro1_xyz = [-173.2,   0,     0]
    hydro2_xyz = [-86.6,  -150.0,     0]
    hydro3_xyz = [-86.6,  -50.0, -100.0]

    max_range = 10000

    xi = np.linspace(-max_range/1000, max_range/1000, (max_range/1000)*2)
    yi = np.linspace(-max_range/1000, max_range/1000, (max_range/1000)*2)
    npts = len(x_list)

    zi = griddata(x_list, y_list, z_list, xi, yi, interp='nn')

    # contour the gridded data, plotting dots at the nonuniform data points.
    if typemeasure == 'Heading':
        levels = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30]
    elif typemeasure == "Declination":
        levels = [0,0.02,0.04,0.06,0.08,0.1,0.12,0.14,0.16,0.18,0.20,0.22,0.24,0.26,0.28]#15#[-1,0,0.05,0.1,0.15,0.2,0.25,0.3,0.35,0.4,0.45,0.5,0.75,1.0,6.30]

    CS = plt.contour(xi, yi, zi, 5, linewidths=0.5, colors='k')

    if typemeasure == 'Heading':
        vmax = 30
        vmin = 0
    elif typemeasure == 'Declination':
        vmax=abs(zi).max()
        vmin=-abs(zi).max()

    CS = plt.contourf(xi, yi, zi, levels, #100,
                      vmax=vmax, vmin=vmin)

    cb = plt.colorbar()
    cb.set_label(label='%s Error (Degrees)' % typemeasure)#,size=18)

    plt.xlim(-max_range/1000, max_range/1000)
    plt.ylim(-max_range/1000, max_range/1000)
    z = abs(z/1000)
    plt.ylabel('Meters')#,size=18)
    plt.xlabel('Meters')#,size=18)

    plt.plot([-hydro0_xyz[0]/100, -hydro1_xyz[0]/100, -hydro2_xyz[0]/100, -hydro3_xyz[0]/100], 
        [-hydro0_xyz[1]/100, -hydro1_xyz[1]/100, -hydro2_xyz[1]/100, -hydro3_xyz[1]/100], 'wo')
    plt.plot([-hydro1_xyz[0]/100, -hydro2_xyz[0]/100, -hydro3_xyz[0]/100], 
        [-hydro1_xyz[1]/100, -hydro2_xyz[1]/100, -hydro3_xyz[1]/100], 'ko', markersize = 3)
    plt.plot([-hydro0_xyz[0]/100], [-hydro0_xyz[1]/100], 'yo', markersize = 3)

    plt.xlim(-max_range/1000, max_range/1000)
    plt.ylim(-max_range/1000, max_range/1000)
    z = abs(z/1000)
    plt.ylabel('Meters')#,size=18)
    plt.xlabel('Meters')#,size=18)
    figure_title = 'Pinger Location VS %s Accuracy' % typemeasure
    figure_sub_title = '%i k/S/s sample rate (%d points) at depth %i meter(s)' % (2000,npts,z)
    

    plt.suptitle('%s\n%s' % (figure_title,figure_sub_title), weight = 'bold', size = 14, x = 0.46, y = 1.01, horizontalalignment='center')

    plt.savefig('Tshape_high_error_resolution_contours_%s_%i_d%i_s%i.png' % (typemeasure,2000,z,npts), dpi=300,
                 orientation = 'landscape', bbox_inches='tight')    

    plt.show()
    plt.clf()
    plt.close()              


X = []
Y = []

#levels = np.linspace(-1, 1, 40)

for distance in range(2,11):
    for bearing in range(360,0,-10):
        rad = math.radians(bearing)
        (x,y) = pol2cart(distance,rad)
        X.append(x)
        Y.append(y)
        print "x: %0.2f y: %0.2f" % (x,y)



x_list = X #[1,-2,3,-4,5,-2,-1,0,1,2]
y_list = Y #[-1,1,0,2,-4,2,1,0,-1,-2]
z = -1000
z_list = [10.26666667, 22.73333333, 17.73333333, 14.7, 13.3, 20.93333333, 21, 16, 11.33333333, 10.46666667,
 13.36666667, 22.26666667, 15.93333333, 8.466666667, 3.366666667, 0.4666666667, 4.266666667, 8.033333333, 
 15.1, 25, 20, 15.23333333, 15.96666667, 18.53333333, 21.33333333, 20.3, 17.36666667, 15.43333333, 15.7, 
 15.6, 11.46666667, 6.733333333, 3.266666667, 2.3, 0.5333333333, 4.166666667,  #2 meters
 3.3, 15.9, 12.96666667, 9.566666667, 1, 11.06666667, 5.866666667, 13.6, 10.5, 9.966666667, 13.63333333,
 21.16666667, 15.8, 8.4, 7.133333333, 0.2666666667, 4.3, 8, 14.96666667, 10, 20, 30, 19.26666667, 
 23.03333333, 22.23333333, 15.13333333, 11.73333333, 8.866666667, 10.53333333, 16.63333333, 12.2, 
 9, 4.433333333, 2.033333333, 0.9666666667, 4.933333333, #3 meters
 12.03333333, 18.93333333, 14.7, 12.53333333, 10.53333333, 17.3, 15.3, 19.86666667, 8.233333333,
 13.2, 17.9, 24.5, 17.26666667, 10.33333333, 0.1333333333, 1, 3.9, 9, 15.3, 10, 20, 20, 25.9,
 22.03333333, 17.63333333, 14.5, 6.966666667, 10.46666667, 12.4, 20, 12.46666667, 7.833333333, 
 5, 1, 7.333333333, 3.9, #4 meters
 15.00, 5.13, 17.27, 9.37, 11.20, 17.73, 10.27, 25.00, 10.00, 5.87, 7.17, 13.73, 21.20, 9.03, 4.67,
 1.07, 2.23, 6.80, 13.00, 10.00, 20.00, 13.30, 14.60, 16.60, 19.17, 11.03, 7.13, 10.63, 10.80, 17.93,
 12.40, 8.13, 1.23, 0.73, 1.20, 5.67, #5 meters
 7.7, 20.3, 15.63333333, 12.93333333, 10.66666667, 13.06666667, 10.5, 15, 10, 2.5, 20, 10.53333333, 14.56666667, 
 7.933333333, 4.933333333, 1.4, 2.166666667, 6.233333333, 0, 10, 20, 6.6, 15.66666667, 13.23333333, 16.86666667,
 11, 9.366666667, 9.666666667, 13.03333333, 16.96666667, 10.6, 5.966666667, 3.2, 0, 13.3, 3.8, #6 meters
 6.733333333, 15.6, 15.8, 13.16666667, 9.566666667, 9.966666667, 10.93333333, 14.63333333, 5, 0, 20, 3.9,
 13.56666667, 6.366666667, 2.666666667, 4.933333333, 0.5333333333, 6.266666667, 0, 10, 10, 5.333333333, 30, 
 11.63333333, 14.2, 13.56666667, 9.733333333, 9.7, 10.56666667, 17.73333333, 6.966666667, 7.033333333, 10,
 0.1333333333, 5.333333333, 6.4, #7 meters
 0, 5, 5, 0, 5, 5, 15, 25, 5, 25, 5, 5, 3.5, 14.16666667, 1.133333333, 7.733333333, 12.36666667,
 9.433333333, 5, 10, 25, 30, 16, 3.666666667, 16.03333333, 13.16666667, 11.03333333, 10.43333333, 
 12.96666667, 7.366666667, 0, 4.6, 5, 0, 5, 10, #8 meters
 15, 25, 9.233333333, 30, 5, 25, 1.366666667, 5, 5, 0, 5, 5, 2.833333333, 17.86666667, 11.46666667,
 30, 30, 15, 0, 5.566666667, 10, 30, 15, 25, 15, 25.56666667, 25, 9, 4.866666667, 25, 0, 0.5666666667,
 5, 0, 5, 0, #9 meters
 15, 25, 20, 30, 20, 25, 15, 5, 5, 0, 5, 5, 15, 25, 6.333333333, 10.56666667, 12.7, 20, 10, 5, 5,
 0, 5, 0, 14.23333333, 14.43333333, 7.533333333, 30, 10.2, 18.16666667, 15, 0, 5, 0, 5, 0] #10 meters



#[0,0.1,0.2,0,0.2,0.15,0.1,0,0.005,0.1]

plot_grid_graph(x_list,y_list,z,z_list,'Heading')

'''fig, axs = plt.subplots(1,1)



Z = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]'''


'''	[10.26666667, 22.73333333, 17.73333333, 14.7, 13.3, 20.93333333, 21, 16, 11.33333333, 10.46666667,
 13.36666667, 22.26666667, 15.93333333, 8.466666667, 3.366666667, 0.4666666667, 4.266666667, 8.033333333, 
 15.1, 25, 20, 15.23333333, 15.96666667, 18.53333333, 21.33333333, 20.3, 17.36666667, 15.43333333, 15.7, 
 15.6, 11.46666667, 6.733333333, 3.266666667, 2.3, 0.5333333333, 4.166666667])'''


'''x = np.linspace(0, 10, 360)


X, Y = np.meshgrid(x, x)
Z = np.sin(X)*np.sin(Y)

levels = np.linspace(-1, 1, 40)

zdata = np.sin(8*X)*np.sin(8*Y)

cs = axs.contourf(X, Y, zdata, levels=levels)
fig.colorbar(cs, ax=axs, format="%.2f")

'''