T = 0.1;
rho0 = 3;
nAttract = 2;
nRepulse = 1;
xObstacle = [2,6,8];
yObstacle = [2.5,5,8];
xTarget = 10;
yTarget = 10;
x = 0;
y = 0;

for i = 2:100 % Run algorithm for 100 sample periods
  % Distance to an obstacle (Eqn 8)
  for j = 1:3
    rho(j) = sqrt( ((yObstacle(j)-y(i-1))^2) + ((xObstacle(j)-x(i-1))^2) );
    % Calculate gradient (Eqns 11 and 13)
    if rho(j) < rho0
      ur_x(j) = nRepulse*(xObstacle(j)-x(i-1))*((1/rho(j))-(1/rho0))/(rho(j)^3);
      ur_y(j) = nRepulse*(yObstacle(j)-y(i-1))*((1/rho(j))-(1/rho0))/(rho(j)^3);
    else
      ur_x(j) = 0;
      ur_y(j) = 0;
    endif
  endfor
  % Calculate new robot position (Eqns 15 and 16)
  x(i) = x(i-1)-T*nAttract*(x(i-1)-xTarget)-T*ur_x(1)-T*ur_x(2)-T*ur_x(3);
  y(i) = y(i-1)-T*nAttract*(y(i-1)-yTarget)-T*ur_y(1)-T*ur_y(2)-T*ur_y(3);
endfor

plot(x, y);