#include "pinger_planar.h"
#include <stdio.h>

// function returns 0 if no real solutions are found

int find_pinger_locations_planar(double *x, double *y, double *zsquared, struct hydrophone_data_planar *h_data, struct distance_data *d_data)
{
	double x1 = h_data->x1 ;
	double x2 = h_data->x2 ;
	double y2 = h_data->y2 ;
	double x3 = h_data->x3 ;
	double y3 = h_data->y3 ;

	double del1 = d_data->del1 ;
	double del2 = d_data->del2 ;
	double del3 = d_data->del3 ;

	double A1, B1, D1 ;
	double A2, B2, D2 ;

	A1 = x1/del1 - x2/del2 ;   // eqn (12)
	B1 = -y2/del2 ;
	D1 = (x2*x2 + y2*y2-del2*del2)/(2.0*del2) - (x1*x1-del1*del1)/(2*del1) ;

	A2 = x1/del1 - x3/del3 ;   // eqn (14)
	B2 = -y3/del3 ;
	D2 = (x3*x3 + y3*y3-del3*del3)/(2.0*del3) - (x1*x1-del1*del1)/(2*del1) ;

	*x =  (B1*D2-B2*D1)/(A1*B2-A2*B1) ;  // eqn (15)
	*y = -(A1*D2-A2*D1)/(A1*B2-A2*B1) ;

	double myx, myy ;
	myx = *x ;
	myy = *y ;

	double T1, T2 ;
	T1 = -4*del1*del1 ;
	T2 =  4*(x1*x1-del1*del1)*myx*myx + 4*x1*(del1*del1-x1*x1)*myx + del1*del1*del1*del1 -2*del1*del1*x1*x1 -4*del1*del1*myy*myy + x1*x1*x1*x1 ;

	*zsquared = -T2/T1 ;

/*
	double x_ans, y_ans, z_ans, d0_ans, d1_ans, d2_ans, d3_ans, del1_ans, del2_ans, del3_ans ;
	x_ans =  1.5 ;
	y_ans =  6.7 ;
	z_ans =  7.75 ;
	printf("A1x+B1y+D1= %8.6lf\n", A1*x_ans+B1*y_ans+D1) ;
	printf("A2x+B2y+D2= %8.6lf\n", A2*x_ans+B2*y_ans+D2) ;
	d0_ans = sqrt(x_ans*x_ans + y_ans*y_ans + z_ans*z_ans) ;
	printf("d0 = %8.6lf\n", d0_ans) ;
	d1_ans = sqrt((x_ans-x1)*(x_ans-x1) + y_ans*y_ans + z_ans*z_ans) ;
	printf("d1 = %8.6lf\n", d1_ans) ;
*/

	return 1 ;

}