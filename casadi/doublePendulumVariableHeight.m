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
                   
desiredTimings = [0.6; 1.2; 0.8; 1.2; 0.6]; 

desiredLegLength = 1.18;


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

rhs1 = [vx;
    vy;
    vz;
    (px - xL1(1) - x_copL) * ul + (px - xR1(1) - x_copR) * ur;
    (py - xL1(2) - y_copL) * ul + (py - xR1(2) - y_copR) * ur;
    -9.81 + (pz - xL1(3)) * ul + (pz - xR1(3)) * ur];

rhs2 = [vx;
    vy;
    vz;
    (px - xR1(1) - x_copR) * ur;
    (py - xR1(2) - y_copR) * ur;
    -9.81 + (pz - xR1(3)) * ur];

rhs3 = [vx;
    vy;
    vz;
    (px - xL2(1) - x_copL) * ul + (px - xR1(1) - x_copR) * ur;
    (py - xL2(2) - y_copL) * ul + (py - xR1(2) - y_copR) * ur;
    -9.81 + (pz - xL2(3)) * ul + (pz - xR1(3)) * ur];

rhs4 = [vx;
    vy;
    vz;
    (px - xL2(1) - x_copL) * ul;
    (py - xL2(2) - y_copL) * ul;
    -9.81 + (pz - xL2(3)) * ul];

rhs5 = [vx;
    vy;
    vz;
    (px - xL2(1) - x_copL) * ul + (px - xR2(1) - x_copR) * ur;
    (py - xL2(2) - y_copL) * ul + (py - xR2(2) - y_copR) * ur;
    -9.81 + (pz - xL2(3)) * ul + (pz - xR2(3)) * ur];

x = [px; py; pz; vx; vy; vz];
u = [x_copL; y_copL; ul; x_copR; y_copR; ur];
dtInt = MX.sym('dt');

F1 = createDiscretizedFunction('F1', rhs1, x, u, dtInt);
F2 = createDiscretizedFunction('F2', rhs2, x, u, dtInt);
F3 = createDiscretizedFunction('F3', rhs3, x, u, dtInt);
F4 = createDiscretizedFunction('F4', rhs4, x, u, dtInt);
F5 = createDiscretizedFunction('F5', rhs5, x, u, dtInt);

pxFoot = MX.sym('pxf');
pyFoot = MX.sym('pyf');
pzFoot = MX.sym('pzf');
pFoot = [pxFoot; pyFoot; pzFoot];

xCop = MX.sym('x_cop');
yCop = MX.sym('y_cop');
u = MX.sym('u_generic');
foot_cop = [xCop; yCop];
foot_control = [foot_cop;u];

frictionBounds = [staticFriction^2;
                  torsionalFriction;
                  torsionalFriction];
    
torsional_multiplier = [(py - pyFoot)/(pz - pzFoot), ...
                       -(px - pxFoot)/(pz - pzFoot)];
              
              
friction_value = [((px - pxFoot - xCop)/(pz - pzFoot))^2 + ...
                 ((py - pyFoot - yCop)/(pz - pzFoot))^2;
                 torsional_multiplier * foot_cop;
                 -torsional_multiplier * foot_cop];
                 
                 
friction_function = Function('friction', {x, foot_cop, pFoot}, {friction_value});

controlLimitsVector = [-copLimits(1,1);
                        copLimits(1,2);
                       -copLimits(2,1);
                        copLimits(2,2)
                        0];

controlLimitValue = [-xCop;
                      xCop;
                     -yCop;
                      yCop;
                     -u];
              
controlBoundsfunction = Function('cop_bounds', {foot_control}, {controlLimitValue});

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
  opti.subject_to(X(:,k+1)==F1(X(:,k),U(:,k), dt));
  
  opti.subject_to((X(1:3,k+1) - xL1)' * (X(1:3,k+1) - xL1) <= legLength^2 );   
  opti.subject_to(friction_function(X(:,k+1), U(1:2,k), xL1) <= frictionBounds);
  opti.subject_to(controlBoundsfunction(U(1:3,k)) <= controlLimitsVector);
    
  opti.subject_to((X(1:3,k+1) - xR1)' * (X(1:3,k+1) - xR1) <= legLength^2 );  
  opti.subject_to(friction_function(X(:,k+1), U(4:5,k), xR1) <= frictionBounds);
  opti.subject_to(controlBoundsfunction(U(4:6,k)) <= controlLimitsVector);
  
  torquesCost = torquesCost + ((X(3, k+1) - xL1(3) - desiredLegLength)*U(3,k))^2;
  torquesCost = torquesCost + ((X(3, k+1) - xR1(3) - desiredLegLength)*U(6,k))^2;

end

for k=phase_length + 1 : 2 * phase_length
  dt = T(2)/(N/5);
  opti.subject_to(X(:,k+1)==F2(X(:,k),U(:,k), dt));
  
  opti.subject_to(U(1:3,k) == zeros(3,1));
  
  opti.subject_to((X(1:3,k+1) - xR1)' * (X(1:3,k+1) - xR1) <= legLength^2 );
  opti.subject_to(friction_function(X(:,k), U(4:5,k), xR1) <= frictionBounds);
  opti.subject_to(controlBoundsfunction(U(4:6,k)) <= controlLimitsVector);
  
  torquesCost = torquesCost + ((X(3, k+1) - xR1(3) - desiredLegLength)*U(6,k))^2;
end

for k= 2 * phase_length + 1 : 3 * phase_length
  dt = T(3)/(N/5);
  opti.subject_to(X(:,k+1)==F3(X(:,k),U(:,k), dt));
  
  opti.subject_to((X(1:3,k+1) - xL2)' * (X(1:3,k+1) - xL2) <= legLength^2 );  
  opti.subject_to(friction_function(X(:,k), U(1:2,k), xL2) <= frictionBounds);
  opti.subject_to(controlBoundsfunction(U(1:3,k)) <= controlLimitsVector);
  
  opti.subject_to((X(1:3,k+1) - xR1)' * (X(1:3,k+1) - xR1) <= legLength^2 );
  opti.subject_to(friction_function(X(:,k), U(4:5,k), xR1) <= frictionBounds);
  opti.subject_to(controlBoundsfunction(U(4:6,k)) <= controlLimitsVector);
  
  torquesCost = torquesCost + ((X(3, k+1) - xL2(3) - desiredLegLength)*U(3,k))^2;
  torquesCost = torquesCost + ((X(3, k+1) - xR1(3) - desiredLegLength)*U(6,k))^2;
end

for k= 3 * phase_length + 1 : 4 * phase_length
  dt = T(4)/(N/5);
  opti.subject_to(X(:,k+1)==F4(X(:,k),U(:,k), dt));
  
  opti.subject_to((X(1:3,k+1) - xL2)' * (X(1:3,k+1) - xL2) <= legLength^2 );
  opti.subject_to(friction_function(X(:,k), U(1:2,k), xL2) <= frictionBounds);
  opti.subject_to(controlBoundsfunction(U(1:3,k)) <= controlLimitsVector);
  
  opti.subject_to(U(4:6,k) == zeros(3,1));
  
  torquesCost = torquesCost + ((X(3, k+1) - xL2(3) - desiredLegLength)*U(3,k))^2;  
end

for k= 4 * phase_length + 1 : 5 * phase_length
  dt = T(5)/(N/5);
  opti.subject_to(X(:,k+1)==F5(X(:,k),U(:,k), dt));

  opti.subject_to((X(1:3,k+1) - xL2)' * (X(1:3,k+1) - xL2) <= legLength^2 );
  opti.subject_to(friction_function(X(:,k), U(1:2,k), xL2) <= frictionBounds);
  opti.subject_to(controlBoundsfunction(U(1:3,k)) <= controlLimitsVector);
  
  opti.subject_to((X(1:3,k+1) - xR2)' * (X(1:3,k+1) - xR2) <= legLength^2 );
  opti.subject_to(friction_function(X(:,k), U(4:5,k), xR2) <= frictionBounds);
  opti.subject_to(controlBoundsfunction(U(4:6,k)) <= controlLimitsVector);
  
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
