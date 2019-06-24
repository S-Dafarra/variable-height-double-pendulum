close all 
clear all

xL1 = [0.0; 0.15; 0.0];
xR1 = [0.0; -0.15; 0.0];
xL2 = [0.6; 0.15; 0.4];
xR2 = [0.6; -0.15; 0.4];


initialPosition = [0.0; 0.0; 1.16];
initialVelocity = [0.0; 0.0; 0.0];

finalPosition = [0.6; 0.0; 1.56];
finalVelocity = [0.0; 0.0; 0.0];

copLimits = [-0.05, 0.05;
             -0.05, 0.05];

legLength = 1.20;

staticFriction = 0.5;
torsionalFriction = 0.03;

desiredFinalControl = [0.0;
                       0.0;
                       9.81/(2*(finalPosition(3) - xL2(3)));
                       0.0;
                       0.0;
                       9.81/(2*(finalPosition(3) - xR2(3)))];

desiredLegLength = 1.18;
                   
desiredTimings = [0.6; 1.2; 0.8; 1.2; 0.6]; 

phases = [true, true;
         false, true;
         true, true;
         true, false;
         true, true;];
     
feetLocations = {xL1, xR1;
                 xL1, xR1;
                 xL2, xR1;
                 xL2, xR1;
                 xL2, xR2;};                
     
assert(length(desiredTimings) == size(phases, 1));
assert(size(phases,2) == 2);

import casadi.*

px = MX.sym('px');
py = MX.sym('py');
pz = MX.sym('pz');

vx = MX.sym('vx');
vy = MX.sym('vy');
vz = MX.sym('vz');

x_copL = MX.sym('x_copL');
y_copL = MX.sym('y_copL');
x_copR = MX.sym('x_copR');
y_copR = MX.sym('y_copR');

ul = MX.sym('ul');
ur = MX.sym('ur');

x = [px; py; pz; vx; vy; vz];
u = [x_copL; y_copL; ul; x_copR; y_copR; ur];
dtInt = MX.sym('dt');

numberOfPhases = size(phases,1);
for k = 1 : numberOfPhases
   F{k} = getPhaseDependentDynamics(strcat('F',num2str(k)), phases(k,:), ...
                                    feetLocations{k,1},feetLocations{k,2},...
                                    x, u, dtInt);
end

pxFoot = MX.sym('pxf');
pyFoot = MX.sym('pyf');
pzFoot = MX.sym('pzf');
pFoot = [pxFoot; pyFoot; pzFoot];

xCop = MX.sym('x_cop');
yCop = MX.sym('y_cop');
u = MX.sym('u_generic');
foot_cop = [xCop; yCop];
foot_control = [foot_cop;u];

[constraints, bounds] = getConstraints('constraints', pFoot, copLimits, ...
                                       legLength, staticFriction, ...
                                       torsionalFriction, x, foot_control);

opti = casadi.Opti();

phase_length = 30;
N = 5 * phase_length;

X = opti.variable(6, N + 1);
U = opti.variable(6, N);

T = opti.variable(5);

opti.subject_to(X(:,1) == [initialPosition; initialVelocity]);

torquesCost = MX.zeros(1);

for k=1:phase_length
  dt = T(1)/(N/5);
  opti.subject_to(X(:,k+1)==F{1}(X(:,k),U(:,k), dt));
  
  opti.subject_to(constraints(X(:,k+1), U(1:3,k), xL1) <= bounds); 
  opti.subject_to(constraints(X(:, k+1), U(4:6, k), xR1) <= bounds);
  
  torquesCost = torquesCost + ((X(3, k+1) - xL1(3) - desiredLegLength)*U(3,k))^2;
  torquesCost = torquesCost + ((X(3, k+1) - xR1(3) - desiredLegLength)*U(6,k))^2;

end

for k=phase_length + 1 : 2 * phase_length
  dt = T(2)/(N/5);
  opti.subject_to(X(:,k+1)==F{2}(X(:,k),U(:,k), dt));
  
  opti.subject_to(U(1:3,k) == zeros(3,1));
  opti.subject_to(constraints(X(:, k+1), U(4:6, k), xR1) <= bounds);
  
  torquesCost = torquesCost + ((X(3, k+1) - xR1(3) - desiredLegLength)*U(6,k))^2;
end

for k= 2 * phase_length + 1 : 3 * phase_length
  dt = T(3)/(N/5);
  opti.subject_to(X(:,k+1)==F{3}(X(:,k),U(:,k), dt));
  
  opti.subject_to(constraints(X(:,k+1), U(1:3,k), xL2) <= bounds); 
  opti.subject_to(constraints(X(:, k+1), U(4:6, k), xR1) <= bounds);
  
  torquesCost = torquesCost + ((X(3, k+1) - xL2(3) - desiredLegLength)*U(3,k))^2;
  torquesCost = torquesCost + ((X(3, k+1) - xR1(3) - desiredLegLength)*U(6,k))^2;
end

for k= 3 * phase_length + 1 : 4 * phase_length
  dt = T(4)/(N/5);
  opti.subject_to(X(:,k+1)==F{4}(X(:,k),U(:,k), dt));
  
  opti.subject_to(constraints(X(:,k+1), U(1:3,k), xL2) <= bounds);  
  opti.subject_to(U(4:6,k) == zeros(3,1));
  
  torquesCost = torquesCost + ((X(3, k+1) - xL2(3) - desiredLegLength)*U(3,k))^2;  
end

for k= 4 * phase_length + 1 : 5 * phase_length
  dt = T(5)/(N/5);
  opti.subject_to(X(:,k+1)==F{5}(X(:,k),U(:,k), dt));

  opti.subject_to(constraints(X(:,k+1), U(1:3,k), xL2) <= bounds); 
  opti.subject_to(constraints(X(:, k+1), U(4:6, k), xR2) <= bounds);
  
  torquesCost = torquesCost + ((X(3, k+1) - xL2(3) - desiredLegLength)*U(3,k))^2;
  torquesCost = torquesCost + ((X(3, k+1) - xR2(3) - desiredLegLength)*U(6,k))^2;
end

opti.subject_to(T >= 0.5 * ones(5,1))
opti.subject_to(T <= 2 * ones(5,1))
opti.set_initial(T, desiredTimings);
opti.set_initial(X(3, :), linspace(initialPosition(3), finalPosition(3), N+1));

opti.minimize((T - desiredTimings)' * (T - desiredTimings) ...
    + 100 * (X(:,end) - [finalPosition; finalVelocity])' * (X(:,end) - [finalPosition; finalVelocity]) ...
    + 0.1/N * (sumsqr(U(3,:)) + sumsqr(U(6,:))) ...
    + 100/N * (sumsqr(U(1:2,:)) + sumsqr(U(4:5,:))) ...
    + 1/N * sumsqr(U(:,2:end) - U(:,1:end-1)) ...
    + 1 * ((U(:,end) - desiredFinalControl)' * (U(:,end) - desiredFinalControl)) ...
    + 1/N * torquesCost);

opti.solver('ipopt');

sol = opti.solve();

plotOptiSolutionForDoublePendulum(sol, X, U, T, phase_length);
