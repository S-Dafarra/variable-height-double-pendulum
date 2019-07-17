function [u, p] = controller(state, desiredState, Kp, Kd)
    
w = [state(1:3), -[1;0;0], -[0;1;0]];

desiredAcceleration = -[Kp, Kd] * (state - desiredState);

U = w\(desiredAcceleration + [0;0;9.81]);

u = U(1);
if (abs(u) < 0.001)
    p = zeros(2,1);
else
    p = U(2:3)/u;
end
end