function F = createDiscretizedFunction(name, rhs, x, u, dtInt)
import casadi.*
f = Function(strcat(name, 'dynamics'), {x, u}, {rhs});
k1 = f(x, u);
k2 = f(x + dtInt/2 * k1, u);
k3 = f(x + dtInt/2 * k2, u);
k4 = f(x + dtInt * k3, u);
xf = x+dtInt/6*(k1 +2*k2 +2*k3 +k4);

F = Function(name, {x, u, dtInt}, {xf});