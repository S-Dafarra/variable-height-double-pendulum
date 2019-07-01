function [xsol, usol, t_sol] = solveVariableHeightDoublePendulum(initialState, ...
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
dtInt = casadi.MX.sym('dt');

[constraintsFcn, bounds] = getConstraints('constraints', constraints.cop, ...
                                           constraints.legLength, constraints.staticFriction, ...
                                           constraints.torsionalFriction);

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

opti.subject_to(X(:,1) == [initialState.position; initialState.velocity]);

torquesCost = casadi.MX.zeros(1);

for phase = 1 : numberOfPhases
    
    dt = T(phase)/phase_length;

    for k = (phase - 1) * phase_length + 1 : phase * phase_length
        opti.subject_to(X(:,k+1)==F{phase}(X(:,k),U(:,k), dt));
        
        if (activeFeet(phase, 1))
            opti.subject_to(constraintsFcn(X(:,k+1), U(1:3,k), feetLocations{phase,1}) <= bounds);
            torquesCost = torquesCost + ((X(3, k+1) - feetLocations{phase,1}(3) - references.legLength)*U(3,k))^2;
        else
            opti.subject_to(U(1:3,k) == zeros(3,1));
        end
        
        if (activeFeet(phase, 2))
            opti.subject_to(constraintsFcn(X(:,k+1), U(4:6,k), feetLocations{phase,2}) <= bounds);
            torquesCost = torquesCost + ((X(3, k+1) - feetLocations{phase,2}(3) - references.legLength)*U(6,k))^2;
        else
            opti.subject_to(U(4:6,k) == zeros(3,1));
        end        
    end
    
end

opti.subject_to(T >= constraints.minimumTimings);
opti.subject_to(T <= constraints.maximumTimings);

opti.set_initial(T, references.timings);
opti.set_initial(X(1, 2:end), linspace(initialState.position(1), references.state.position(1), N));
opti.set_initial(X(3, 2:end), linspace(initialState.position(3), references.state.position(3), N));
opti.set_initial(X(:,1), [initialState.position; initialState.velocity]);

opti.minimize(weights.time * (T - references.timings)' * (T - references.timings) ...
    + weights.finalState * sumsqr(X(:,end - round(phase_length * references.state.anticipation) : end) - [references.state.position; references.state.velocity]) ...
    + weights.u * (sumsqr(U(3,:)) + sumsqr(U(6,:))) ...
    + weights.cop * (sumsqr(U(1:2,:)) + sumsqr(U(4:5,:))) ...
    + weights.controlVariation * sumsqr(U(:,2:end) - U(:,1:end-1)) ...
    + weights.finalControl * ((U(:,end) - references.control)' * (U(:,end) - references.control)) ...
    + weights.torques * torquesCost);

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
