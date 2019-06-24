function x_dot = variableHeightPendulum(~, x, u, p)
x_dot = zeros(6,1);
x_dot(1:3) = x(4:6);
x_dot(4:6) = -[0.0;0.0;9.81] + (x(1:3) - [p;0]) * u;
end