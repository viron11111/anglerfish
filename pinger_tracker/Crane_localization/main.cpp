#include <stdio.h>
#include "pinger_planar.h"

void main()
{
	FILE *fp ;


	int find_pinger_locations_planar(double *x, double *y, double *zsquared, struct hydrophone_data_planar *h_data, struct distance_data *d_data) ;

	// Set values for input items.


	struct hydrophone_data_planar h_data ;
	h_data.x1 =  8.5 ;  // units of length
	h_data.x2 =  5.65 ;
	h_data.y2 =  4.75 ;
	h_data.x3 = -1.75 ;
	h_data.y3 =  3.5 ;

	struct distance_data d_data ;
	d_data.del1 =  2.053896 ; // units of length
	d_data.del2 = -1.349004 ;
	d_data.del3 = -1.361367 ;  //-1.361367 ;

	// Declare output variables.
	double Px ;
	double Py ;
	double Pz_squared ;
	double Pz ;

	int ok ;

	// Call function.
  	ok = find_pinger_locations_planar(&Px, &Py, &Pz_squared, &h_data, &d_data) ;

	Pz = sqrt(Pz_squared) ;

	// Print results to a file.
	fp = fopen("pinger_planar_out.txt","w") ;

	// print input values to file
	fprintf(fp, "INPUT VALUES:\n") ;
	fprintf(fp, "\tx1 = %8.6lf\n", h_data.x1) ;
	fprintf(fp, "\tx2 = %8.6lf\ty2 = %8.6lf\n", h_data.x2, h_data.y2) ;
	fprintf(fp, "\tx3 = %8.6lf\ty3 = %8.6lf\n", h_data.x3, h_data.y3) ;
	fprintf(fp, "\tdel1 = %8.6lf\tdel2 = %8.6lf\tdel3 = %8.6lf\n\n", d_data.del1, d_data.del2, d_data.del3) ;

	// print output values to file
	fprintf(fp, "OUTPUT VALUES:\n") ;
	if(!ok)
	{
		// no real results were found
		fprintf(fp, "\tNo real results were found.\n\n") ;
	}
	else
	{
		fprintf(fp, "\tx = %8.6lf\ty = %8.6lf\tzsquared = %8.6lf\n",  Px, Py, Pz_squared) ;
		if(Pz_squared>=0)
			fprintf(fp, "\t\tPz1 = %8.6lf\n", sqrt(Pz_squared)) ;
		fprintf(fp, "\n") ;
	}

	// Check results.  Calculate d0, d1, d2, d3 and get del1, del2, del3
	double check_d0, check_d1, check_d2, check_d3 ;
	double check_del1, check_del2, check_del3 ;
	check_d0 = sqrt(Px*Px+Py*Py+Pz*Pz) ;
	check_d1 = sqrt((Px-h_data.x1)*(Px-h_data.x1)+Py*Py+Pz*Pz) ;
	check_d2 = sqrt((Px-h_data.x2)*(Px-h_data.x2)+(Py-h_data.y2)*(Py-h_data.y2)+Pz*Pz) ;
	check_d3 = sqrt((Px-h_data.x3)*(Px-h_data.x3)+(Py-h_data.y3)*(Py-h_data.y3)+Pz*Pz) ;

	fprintf(fp, "\nCHECK RESULTS:\n\n") ;
	
	fprintf(fp, "\td0 = %8.6lf\td1 = %8.6lf\td2 = %8.6lf\td3 = %8.6lf\n", check_d0, check_d1, check_d2, check_d3) ;
	fprintf(fp, "\tdel1 = %8.6lf\tdel2 = %8.6lf\tdel3 = %8.6lf\n\n", check_d1-check_d0, check_d2-check_d0, check_d3-check_d0) ;


	fclose(fp) ;

}


