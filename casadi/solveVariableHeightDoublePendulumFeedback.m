function [xsol, usol, a_sol, t_sol, K_sol, x_des_sol] = ...
    solveVariableHeightDoublePendulumFeedback(initialState, ...
                                              references, ...
                                              constraints, ...
                                              activeFeet, ...
                                              feetLocations, ...
                                              phase_length, ...
                                              weights)

                                                        
assert(length(references.timings) == size(activeFeet, 1));
assert(size(activeFeet, 1) == size(feetLocations, 1));
assert(size(activeFeet,2) == 2);
assert(size(feetLocations,2) == 2);

x = casadi.MX.sym('x', 6);
u = casadi.MX.sym('u', 6);
a = casadi.MX.sym('a', 3);
dtInt = casadi.MX.sym('dt');

[footConstraints, bounds] = getConstraints('constraints', constraints.cop, ...
                                           constraints.legLength, constraints.staticFriction, ...
                                           constraints.torsionalFriction);
numberOfPhases = size(activeFeet,1);

for k = 1 : numberOfPhases
   accelerationConstraints{k} = ...
        accelerationConsistencyConstraint(strcat('accelerationConsistency',num2str(k)),...
                                          activeFeet(k,:), ...
                                          feetLocations{k,1}, feetLocations{k,2},...
                                          x, u, a);
end

F = getIntegratorDynamics(x, a, dtInt);

opti = casadi.Opti();

N = numberOfPhases * phase_length;

X = opti.variable(6, N + 1);
A = opti.variable(3,N);
U = opti.variable(6, N);

T = opti.variable(numberOfPhases);

K  = opti.variable(6, numberOfPhases);
x_des = opti.variable(6, numberOfPhases);

opti.subject_to(X(:,1) == [initialState.position; initialState.velocity]);

torquesCost = casadi.MX.zeros(1);
simplifiedControlCost = casadi.MX.zeros(1);

g = [0;0;9.81]; 

for phase = 1 : numberOfPhases
    
    dt = T(phase)/phase_length;
    
    gainMatrix = [diag(K(1:3,phase)), diag(K(4:6,phase))];

    for k = (phase - 1) * phase_length + 1 : phase * phase_length
        opti.subject_to(X(:,k+1)==F(X(:,k),A(:,k), dt));
        
        if (activeFeet(phase, 1))
            opti.subject_to(footConstraints(X(:,k+1), U(1:3,k), feetLocations{phase,1}) <= bounds);
            torquesCost = torquesCost + ((X(3, k+1) - feetLocations{phase,1}(3) - references.legLength)*U(3,k))^2;
        else
            opti.subject_to(U(1:3,k) == zeros(3,1));
        end
        
        if (activeFeet(phase, 2))
            opti.subject_to(footConstraints(X(:,k+1), U(4:6,k), feetLocations{phase,2}) <= bounds);
            torquesCost = torquesCost + ((X(3, k+1) - feetLocations{phase,2}(3) - references.legLength)*U(6,k))^2;
        else
            opti.subject_to(U(4:6,k) == zeros(3,1));
        end
        
        opti.subject_to(accelerationConstraints{phase}(X(:,k + 1), U(:,k), A(:,k)) == 0);
        
        %opti.subject_to(A(:,k) ==  g - gainMatrix * (X(:,k) - x_des(:, phase)));
        simplifiedControlCost = simplifiedControlCost + ...
            (A(:,k) - g + gainMatrix * (X(:,k) - x_des(:, phase)))' * ...
            (A(:,k) - g + gainMatrix * (X(:,k) - x_des(:, phase))); %X(:,k) is supposed to be feedback
    end
    
    opti.subject_to(K(:, phase) >= 0.01);
    opti.subject_to(K(4:6, phase) >= 2*sqrt(K(1:3, phase)));

    
end

opti.subject_to(x_des(:, numberOfPhases) == [references.state.position; references.state.velocity]);

opti.subject_to(T >= constraints.minimumTimings);
opti.subject_to(T <= constraints.maximumTimings);

opti.set_initial(T, references.timings);
points = linspace(0,1,N);
position_guess = initialState.position + points.*(references.state.position - initialState.position);

opti.set_initial(X(1, 2:end), position_guess(1,:));
opti.set_initial(X(3, 2:end), position_guess(3,:));
opti.set_initial(X(:,1), [initialState.position; initialState.velocity]);

for phase = 1 : numberOfPhases
    if (phase ~= numberOfPhases)
        meanPosition = mean(position_guess(1:3,(phase-1) * phase_length + 1 : phase * phase_length), 2);
        opti.set_initial(x_des(1:3, phase),meanPosition);
    end
end

opti.set_initial(x_des(:, numberOfPhases), [references.state.position; references.state.velocity]);
opti.set_initial(K, ones(6, numberOfPhases));

opti.minimize(weights.time * (T - references.timings)' * (T - references.timings) ...
    + weights.finalState * sumsqr(X(:,end - round(phase_length * references.state.anticipation) : end) - [references.state.position; references.state.velocity]) ...
    + weights.u * (sumsqr(U(3,:)) + sumsqr(U(6,:))) ...
    + weights.cop * (sumsqr(U(1:2,:)) + sumsqr(U(4:5,:))) ...
    + weights.controlVariation * sumsqr(U(:,2:end) - U(:,1:end-1)) ...
    + weights.finalControl * ((U(:,end) - references.control)' * (U(:,end) - references.control)) ...
    + weights.torques * torquesCost ...
    + weights.simplifiedControl * simplifiedControlCost ...
    + weights.K * sumsqr(K) ...
    + weights.computedDesired * sumsqr(x_des) ...
    + weights.K_diff * sumsqr(K(:, 2:end) - K(:, 1:end-1))...
    + weights.computedDesiredDiff * sumsqr(x_des(:, 2:end) - x_des(:, 1:end-1)));

options = struct;
options.expand = true;
% options.ipopt.print_level = 0;
options.ipopt.linear_solver='ma27';
    
opti.solver('ipopt', options);

sol = opti.solve();

xsol = sol.value(X);
usol = sol.value(U);
t_sol = sol.value(T); 
a_sol = sol.value(A);
K_sol = sol.value(K);
x_des_sol = sol.value(x_des);

end
