clear all
close all
clc

kp = 2*eye(3);
kd = 2 * sqrt(kp);

initialHeight = 1.16;
desiredHeight = 1.57;
desiredFinalPosition = [0;0;desiredHeight];

initialPosition = [-0.3; -0.1; initialHeight];

w = [initialPosition, -[1;0;0], -[0;1;0]];

initialVelocity = kd^-1 *(- kp * (initialPosition - desiredFinalPosition) ...
    + [0.0; 0.0; 9.81] - w * [0.7*9.81/initialHeight; 0; 0]);
%initialVelocity = [0.57;0.34;0.0];

initialState = [initialPosition; initialVelocity];

desiredFinalVelocity = [0;0;0];

desiredState = [desiredFinalPosition;desiredFinalVelocity];

control = zeros(1,3);

n = 1000;

states = zeros(6, n);
controls = zeros(3, n);
t = zeros(1, n);

currentState = initialState;

for i = 1 : n
    [u, p] = controller(currentState, desiredState, kp, kd);
    
    [~,x] = ode45(@(t,x)variableHeightPendulum(t, x, u, p), ...
        [0, 0.01], currentState);
    
    currentState = x(end, :)';
    
    controls(:, i) = [u;p];
    states(:, i) = currentState;
    t (i) = 0.01 * i;
end

figure
plot(t, states(1:3, :));
title("CoM Position")
legend(["x", "y", "z"]);
ylabel("[m]");
xlabel("t [s]");

figure
plot(t, states(4:6, :));
title("CoM Velocity")
legend(["x", "y", "z"]);
ylabel("[m/s]");
xlabel("t [s]");


figure
plot(t, controls(1,:));
title("u")
ylabel("[1/s^2]");
xlabel("t [s]");

figure
plot(t, controls(2:3,:));
title("CoP")
legend(["x", "y"]);
ylabel("[m]");
xlabel("t [s]");


