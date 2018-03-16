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
        levels = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30]#15#[-1,0,0.05,0.1,0.15,0.2,0.25,0.3,0.35,0.4,0.45,0.5,0.75,1.0,6.30]

    CS = plt.contour(xi, yi, zi, 5, linewidths=0.5, colors='k')

    if typemeasure == 'Heading':
        vmax = 30
        vmin = 0
    elif typemeasure == 'Declination':
        #vmax = 30
        #vmin = 0
        vmax=30#abs(zi).max()
        vmin=-30#-abs(zi).max()

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

    plt.savefig('SONAR_accuracy_%s_%i_d%i_s%i.png' % (typemeasure,2000,z,npts), dpi=300,
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
z_list = [3.70, 0.50, 2.40, 4.57, 7.10, 9.75, 9.43, 9.35, 9.07, 8.83, 8.47, 7.47,
5.97, 12.47, 11.03, 10.30, 10.93, 13.10, 15.50, 3.60, 5.80, 6.47, 5.90, 5.50,
4.83, 4.53, 15.27, 14.93, 11.32, 14.57, 8.53, 10.04, 6.13, 18.60, 11.10, 3.70, #2 meters
2.23, 1.10, 0.03, 0.94, 2.67, 4.37, 3.80, 4.40, 4.47, 4.20, 4.24, 4.30,
2.54, 7.07, 5.70, 4.04, 4.57, 7.10, 10.07, 1.14, 1.77, 2.00, 2.17, 1.54,
1.04, 0.47, -1, -1, -1, -1, -1, -1, 5.93, 6.43, 4.90, 3.90, #3 meters
5.52, -1, 2.99, 2.22, 0.82, 0.68, -1, -1, 2.18, 2.28, 0.95, 0.35,
9.11, 7.51, 6.38, 6.38, 6.08, 6.78, 8.21, 3.62, 1.95, 0.72, 8.35, 0.92,
1.42, 6.68, 5.45, 4.28, -1, -1, -1, 4.38, 7.22, 6.39, 5.85, 3.95, #4 meters
6.06, 4.36, 3.39, 2.33, 0.96, 3.93, 0.06, 0.84, 1.81, 2.41, 1.07, 2.51,
2.41, 5.34, 4.74, 4.71, 4.64, 9.44, -1, 1.26, 0.46, 0.24, 0.43, 1.26,
1.63, -1, 4.33, 4.99, -1, -1, -1, 4.67, 0.91, 5.69, 8.89, 2.36, #5 meters
5.12, -1, 3.81, 2.81, 1.81, 0.81, -1, -1, 1.99, 1.96, 2.49, 3.52,
-1, 3.19, 4.09, 4.59, 4.29, 4.29, -1, 0.99, 1.26, 0.02, 0.44, -1,
-1, -1, 4.51, 4.98, 5.41, -1, -1, 7.76, -1, 7.78, 7.04, 5.96, #6 meters
5.51, 2.79, 1.09, 0.04, 1.61, 1.81, -1, 4.11, 4.48, 4.74, 5.01, 5.78,
-1, 4.10, 6.94, 7.21, 7.01, 7.21, -1, 4.34, 4.14, 3.44, 2.34, 1.98,
-1, 0.59, 1.49, 2.76, 3.92, 5.31, -1, -1, 6.16, 5.42, 4.16, 3.59, #7 meters
-1, 0.26, 1.48, 0.41, 5.11, 3.75, -1, 4.38, 6.85, 3.88, 2.88, -1, 
-1, -1, 8.15, 8.35, 7.65, 6.95, -1, 4.55, 4.71, 3.85, 2.41, 2.18,
0.51, 0.18, 0.82, 1.82, 3.12, -1, -1, -1, -1, 2.45, 2.69, 2.25, #8 meters
1.91, 6.75, 7.25, 8.28, 8.91, -1, -1, -1, -1, -1, 11.15, -1,
-1, -1, 10.75, 11.21, 11.05, 11.08, 9.87, 9.38, 8.88, 8.21, 7.15, 5.85,
4.61, 4.31, 3.31, 2.51, 1.21, 0.25, -1, -1, 0.59, 0.22, 0.05, 0.51, #9 meters
1.61, 4.34, 5.64, 6.24, 6.97, 7.01, -1, 9.84, 9.94, 10.34, 11.07, -1,
-1, -1, 11.74, 11.91, 11.44, 11.01, -1, -1, 9.41, 8.44, 7.54, 6.34,
-1, 4.37, 3.64, 2.54, -1, -1, -1, -1, 1.19, 0.56, 0.36, 0.03] #10 meters



#[0,0.1,0.2,0,0.2,0.15,0.1,0,0.005,0.1]

plot_grid_graph(x_list,y_list,z,z_list,'Declination')

#plt.savefig('SONAR_accuracy_graph.png', dpi=300, orientation = 'landscape', bbox_inches='tight')

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