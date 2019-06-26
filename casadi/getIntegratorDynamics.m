function F = getIntegratorDynamics(x, a, dT)
v = x(4:6);
g = zeros(3,1);
g(3) = -9.81;
rhs = [v;
       a + g];
   
F = createDiscretizedFunction('integrator', rhs, x, a, dT);
end