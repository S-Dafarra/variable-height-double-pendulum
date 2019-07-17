function G = accelerationConsistencyConstraint(name, leftRightActivated, ...
                                               leftLocation, rightLocation, ...
                                               X, U, A)
   
px = X(1);
py = X(2);
pz = X(3);
x_copL = U(1);
y_copL = U(2);
ul = U(3);
x_copR = U(4);
y_copR = U(5);
ur = U(6);

constraint = A;

if (leftRightActivated(1))
  constraint = constraint - [(px - leftLocation(1) - x_copL) * ul;
                             (py - leftLocation(2) - y_copL) * ul;
                             (pz - leftLocation(3)) * ul];
end

if (leftRightActivated(2))
  constraint = constraint - [(px - rightLocation(1) - x_copR) * ur;
                             (py - rightLocation(2) - y_copR) * ur;
                             (pz - rightLocation(3)) * ur];  
end

G = casadi.Function(name, {X, U, A}, {constraint});
                                           
end