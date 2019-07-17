function F = getIntegratorDynamics(x, a, dT)
p = x(1:3);
v = x(4:6);
g = zeros(3,1);
g(3) = -9.81;
rhs = [p + dT * v + 0.5 * dT^2 * (a + g);
       v + dT * (a + g)];
   
F = casadi.Function('integrator', {x, a, dT}, {rhs});  
end