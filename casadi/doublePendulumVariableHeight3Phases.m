close all 
clear all

jump = false;

xL1 = [0.0; 0.15; 0.0];
xR1 = [0.0; -0.15; 0.0];
xL2 = [0.6; 0.15; 0.4];
xR2 = [0.6; -0.15; 0.4];


initialState.position = [0.0; -0.08; 1.16];
initialState.velocity = [0.1; -0.05; 0.0];

references.state.position = [0.6; 0.08; 1.56];
references.state.velocity = [0.1; 0.05; 0.0];
references.state.anticipation = 0.3;

constraints.cop = [-0.05, 0.05;
                   -0.05, 0.05];
constraints.legLength = 1.20;
constraints.staticFriction = 0.5;
constraints.torsionalFriction = 0.03;

references.control = [0.0;
                      0.0;
                      9.81/(references.state.position(3) - xL2(3));
                      0.0;
                      0.0;
                      0.0];

references.legLength = 1.18;
                   
references.timings = [0.6; 1.2; 0.8;]; 

activeFeet = [false, true;
              true, true;
              true, false;];
          
if (jump)
    activeFeet(2,:) = [false, false];              
end
    
feetLocations = {xL1, xR1;
                 xL2, xR1;
                 xL2, xR1;};            
             
phase_length = 30;

numberOfPhases = size(activeFeet, 1);
N = phase_length * numberOfPhases;

constraints.minimumTimings = 0.5 * ones(numberOfPhases,1);
constraints.maximumTimings = 2.0 * ones(numberOfPhases,1);

weights.time = 1;
weights.finalState = 10;
weights.u = 0.1/N;
weights.uMax = 0.1;
weights.cop = 10/N;
weights.controlVariation = 1/N;
weights.finalControl = 1;
weights.torques = 1/N;

tic
[xsol, usol, a_sol, t_sol] = ...
solveVariableHeightDoublePendulumAsIntegrator(initialState,...
                                              references, ...
                                              constraints, ...
                                              activeFeet, ...
                                              feetLocations, ...
                                              phase_length, ...
                                              weights);
toc

plotOptiSolutionForDoublePendulumPlusAccelerations(xsol, usol, a_sol, t_sol);
