function [xsol, usol, t_sol] = solveVariableHeightDoublePendulum(initialPosition, ...
                                                                 initialVelocity,...
                                                                 finalPosition, ...
                                                                 finalVelocity, ...
                                                                 copLimits, ...
                                                                 legLength, ...
                                                                 staticFriction, ...
                                                                 torsionalFriction, ...
                                                                 desiredFinalControl, ...
                                                                 desiredLegLength, ...
                                                                 desiredTimings, ...
                                                                 activeFeet, ...
                                                                 feetLocations, ...
                                                                 phase_length)

                                                        
assert(length(desiredTimings) == size(activeFeet, 1));
assert(size(activeFeet, 1) == size(feetLocations, 1));
assert(size(activeFeet,2) == 2);
assert(size(feetLocations,2) == 2);

import casadi.*

x = MX.sym('x', 6);
u = MX.sym('u', 6);
dtInt = MX.sym('dt');
pFoot = MX.sym('pFoot', 3);
foot_control = MX.sym('foot_control', 3);

[constraints, bounds] = getConstraints('constraints', pFoot, copLimits, ...
                                       legLength, staticFriction, ...
                                       torsionalFriction, x, foot_control);

numberOfPhases = size(activeFeet,1);

for k = 1 : numberOfPhases
   F{k} = getPhaseDependentDynamics(strcat('F',num2str(k)), activeFeet(k,:), ...
                                    feetLocations{k,1},feetLocations{k,2},...
                                    x, u, dtInt);
end

opti = casadi.Opti();

N = numberOfPhases * phase_length;

X = opti.variable(6, N + 1);
U = opti.variable(6, N);

T = opti.variable(numberOfPhases);

opti.subject_to(X(:,1) == [initialPosition; initialVelocity]);

torquesCost = MX.zeros(1);

for phase = 1 : numberOfPhases
    
    dt = T(phase)/phase_length;

    for k = (phase - 1) * phase_length + 1 : phase * phase_length
        opti.subject_to(X(:,k+1)==F{phase}(X(:,k),U(:,k), dt));
        
        if (activeFeet(phase, 1))
            opti.subject_to(constraints(X(:,k+1), U(1:3,k), feetLocations{phase,1}) <= bounds);
            torquesCost = torquesCost + ((X(3, k+1) - feetLocations{phase,1}(3) - desiredLegLength)*U(3,k))^2;
        else
            opti.subject_to(U(1:3,k) == zeros(3,1));
        end
        
        if (activeFeet(phase, 2))
            opti.subject_to(constraints(X(:,k+1), U(4:6,k), feetLocations{phase,2}) <= bounds);
            torquesCost = torquesCost + ((X(3, k+1) - feetLocations{phase,2}(3) - desiredLegLength)*U(6,k))^2;
        else
            opti.subject_to(U(4:6,k) == zeros(3,1));
        end        
    end
    
end

opti.subject_to(T >= 0.5 * ones(numberOfPhases,1))
opti.subject_to(T <= 2 * ones(numberOfPhases,1))

opti.set_initial(T, desiredTimings);
opti.set_initial(X(1, 2:end), linspace(initialPosition(1), finalPosition(1), N));
opti.set_initial(X(3, 2:end), linspace(initialPosition(3), finalPosition(3), N));
opti.set_initial(X(:,1), [initialPosition; initialVelocity]);

for phase = 1 : numberOfPhases
    initialControl = desiredFinalControl;

    range = (phase - 1) * phase_length + 1 : phase * phase_length;
        
    if (activeFeet(phase, 1) && activeFeet(phase, 2))
        opti.set_initial(U(1:6,k), initialControl);
    else
        if activeFeet(phase, 1)
            initialControl(3) = 2 * initialControl(3);
            opti.set_initial(U(1:3,k), initialControl(4:6));
            opti.set_initial(U(4:6,k), zeros(3,1));
        else
            if activeFeet(phase, 2)
                initialControl(6) = 2 * initialControl(3);
                opti.set_initial(U(1:3,k), zeros(3,1));
                opti.set_initial(U(4:6,k), initialControl(4:6));
            else
                opti.set_initial(U(1:6,k), zeros(6,1));
            end
        end
    end
end

opti.minimize((T - desiredTimings)' * (T - desiredTimings) ...
    + 100 * sumsqr(X(:,end - round(phase_length/3) : end) - [finalPosition; finalVelocity]) ...
    + 0.1/N * (sumsqr(U(3,:)) + sumsqr(U(6,:))) ...
    + 100/N * (sumsqr(U(1:2,:)) + sumsqr(U(4:5,:))) ...
    + 1/N * sumsqr(U(:,2:end) - U(:,1:end-1)) ...
    + 1 * ((U(:,end) - desiredFinalControl)' * (U(:,end) - desiredFinalControl)) ...
    + 1/N * torquesCost);

options = struct;
options.expand = true;
options.ipopt.print_level = 0;
options.ipopt.linear_solver='ma27';
    
opti.solver('ipopt', options);

sol = opti.solve();

xsol = sol.value(X);
usol = sol.value(U);
t_sol = sol.value(T);                                                

end
