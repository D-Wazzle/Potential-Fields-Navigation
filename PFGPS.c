#include <math.h>
#include <stdio.h>

int main()
{
	double T = 0.00002;
	double rho[3];
	double ur_x[3];
	double ur_y[3];
	double rho0 = 3.0;
	double nAttract = 2.0;
	double nRepulse = 1.0;
	double xObstacle[3] = {2.0,6.0,8.0};
	double yObstacle[3] = {2.5,5.0,8.0};
	double xTarget = 10.0;
	double yTarget = 10.0;
	double x = 0.0;
	double y = 0.0;
	int i, j;
	
	for (i = 2; i<250003; i++) // Run algorithm for 200000 sample periods
	{
		// Distance to an obstacle (Eqn 8)
		for (j = 0; j < 3; j++)
		{
			rho[j] = sqrt( pow(yObstacle[j]-y, 2) + pow(xObstacle[j]-x, 2) );
			
			// Calculate gradient (Eqns 11 and 13)
			if (rho[j] < rho0)
			{
				ur_x[j] = nRepulse*(xObstacle[j]-x)*((1/rho[j])-(1/rho0))/pow(rho[j], 3);
				ur_y[j] = nRepulse*(yObstacle[j]-y)*((1/rho[j])-(1/rho0))/pow(rho[j], 3);
			}
			
			else
			{
				ur_x[j] = 0;
				ur_y[j] = 0;
			}
			
		}
		
		// Calculate new robot position (Eqns 15 and 16)
		printf("Current X = %f     ", x);
		x = x-(T*nAttract*(x-xTarget))-T*ur_x[0]-T*ur_x[1]-T*ur_x[2];
		printf("Next X = %f\n", x);
		printf("Current Y = %f     ", y);		
		y = y-(T*nAttract*(y-yTarget))-T*ur_y[0]-T*ur_y[1]-T*ur_y[2];
		printf("Next Y = %f\n\n", y);
	}
	
	return 0;
}

//plot(x, y);
