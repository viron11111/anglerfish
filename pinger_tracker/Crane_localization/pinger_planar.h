#include <math.h>

struct hydrophone_data_planar
	{
		double x1 ;
		double x2 ;
		double y2 ;
		double x3 ;
		double y3 ;
	} ;
	struct distance_data
	{
		double del1 ;  // distance difference d1-d0 (d1 is distance from pinger to point 1, d0 is distance of pinger from point 0
		double del2 ;  // distance difference d2-d0
		double del3 ;  // distance difference d3-d0
	} ;
