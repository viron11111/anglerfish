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
z_list = [6.77, 4.37, 2.27, 1.83, 7.70, 13.27, 16.80, 10.97, 6.93, 3.17, 4.83, 9.07,
9.97, 3.00, 0.97, 2.13, 3.83, 7.20, 12.90, 12.93, 1.10, 3.50, 14.63, 6.63,
9.07, 4.73, 0.33, 12.60, 18.30, 11.17, 16.03, 18.07, 3.47, 7.83, 3.10, 2.03, #2 meters
3.17, 0.53, 3.50, 3.67, 3.60, 1.10, 9.40, 9.50, 6.13, 6.73, 7.93, 12.27,
12.60, 4.47, 10.27, 4.00, 8.10, 5.87, 14.00, 8.40, 4.07, 2.10, 7.60, 8.37,
9.37, 5.00, 5.00, 0.00, 10.00, 3.57, 15.00, 10.00, 5.33, 3.60, 2.77, 2.53, #3 meters
1.50, 0.90, 5.40, 5.27, 6.20, 0.90, 17.23, 13.57, 10.23, 8.53, 11.90, 15.03,
11.97, 5.13, 1.20, 2.00, 4.80, 8.47, 14.30, 15.73, 14.20, 26.53, 40.00, 5.57,
1.20, 5.00, 5.00, 0.00, 10.00, 5.00, 15.00, 9.50, 1.27, 4.20, 6.27, 4.93,#4 meters
0.57, 5.00, 3.13, 3.10, 0.40, 0.27, 0.40, 8.03, 14.83, 4.93, 5.30, 5.53,
7.83, 1.50, 1.13, 2.77, 3.73, 5.90, 10.00, 24.53, 33.50, 30.00, 14.77, 10.00,
28.00, 5.00, 4.00, 9.40, 10.00, 5.00, 5.00, 1.20, 4.27, 3.00, 1.50, 0.90, #5 meters
15.03, 5.00, 0.27, 0.80, 1.53, 1.50, 15.00, 25.00, 13.57, 2.13, 0.27, 0.93,
0.00, 3.13, 2.30, 2.30, 4.63, 6.00, 0.00, 0.10, 1.53, 1.50, 1.07, 0.00,
0.00, 5.00, 4.60, 5.60, 4.10, 5.00, 0.00, 4.53, 10.00, 4.67, 3.20, 19.87, #6 meters
5.20, 0.77, 1.43, 1.23, 2.00, 0.83, 15.00, 1.30, 0.33, 1.63, 0.77, 3.30,
5.00, 6.63, 4.97, 1.43, 2.83, 3.50, 0.00, 0.47, 1.10, 3.13, 3.10, 0.10,
0.00, 2.80, 3.17, 3.97, 5.70, 5.00, 0.00, 10.30, 2.10, 2.40, 1.20, 0.23, #7 meters
1.90, 4.17, 4.23, 2.33, 1.90, 1.10, 15.00, 4.63, 2.20, 0.43, 1.67, 10.00,
0.00, 4.70, 1.23, 2.20, 3.50, 4.00, 0.00, 1.77, 0.73, 1.53, 1.07, 0.13,
0.80, 0.50, 0.33, 0.03, 1.60, 5.00, 0.00, 10.00, 6.90, 4.43, 2.53, 3.67, #8 meters
15.00, 3.37, 1.97, 1.20, 0.27, 1.30, 15.00, 10.00, 1.67, 0.23, 2.23, 10.00,
0.00, 10.00, 4.23, 3.03, 4.13, 4.23, 2.83, 8.53, 5.13, 2.33, 0.13, 1.20,
1.87, 1.73, 2.43, 1.37, 2.13, 1.37, 0.00, 10.00, 5.63, 3.97, 4.17, 4.60, #9 meters
4.57, 2.50, 4.50, 3.27, 0.93, 0.73, 0.00, 10.00, 3.90, 0.70, 4.50, 10.00,
0.00, 10.00, 2.50, 2.50, 2.80, 0.30, 0.00, 10.00, 2.27, 1.77, 0.87, 1.33,
0.00, 1.13, 1.83, 0.57, 20.00, 10.00, 0.00, 10.00, 3.77, 1.93, 2.57, 1.57] #10 meters



#[0,0.1,0.2,0,0.2,0.15,0.1,0,0.005,0.1]

plot_grid_graph(x_list,y_list,z,z_list,'Heading')

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