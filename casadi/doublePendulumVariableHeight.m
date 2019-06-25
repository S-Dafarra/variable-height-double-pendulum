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

activeFeet = [true, true;
              false, true;
              true, true;
              true, false;
              true, true;];
     
feetLocations = {xL1, xR1;
                 xL1, xR1;
                 xL2, xR1;
                 xL2, xR1;
                 xL2, xR2;};            
             
phase_length = 30;

[xsol, usol, t_sol] = solveVariableHeightDoublePendulum(initialPosition, ...
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
                                                        phase_length);

plotOptiSolutionForDoublePendulum(xsol, usol, t_sol, phase_length);
