function F = getPhaseDependentDynamics(phaseName, leftRightActivated, ...
                                       leftLocation, rightLocation, X, U, dT)
import casadi.*

px = X(1);
py = X(2);
pz = X(3);
vx = X(4);
vy = X(5);
vz = X(6);
x_copL = U(1);
y_copL = U(2);
ul = U(3);
x_copR = U(4);
y_copR = U(5);
ur = U(6);

rhs = [vx;
       vy;
       vz;
       0.0;
       0.0;
       -9.81];
if (leftRightActivated(1))
  rhs = rhs + [0.0;
               0.0;
               0.0;
               (px - leftLocation(1) - x_copL) * ul;
               (py - leftLocation(2) - y_copL) * ul;
               (pz - leftLocation(3)) * ul];  
end

if (leftRightActivated(2))
  rhs = rhs + [0.0;
               0.0;
               0.0;
               (px - rightLocation(1) - x_copR) * ur;
               (py - rightLocation(2) - y_copR) * ur;
               (pz - rightLocation(3)) * ur];  
end

F = createDiscretizedFunction(phaseName, rhs, X, U, dT);
end