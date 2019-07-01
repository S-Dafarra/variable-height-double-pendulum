function [G, upperBounds] = getConstraints(name, footLocation, ...
                                           copLimits, legLength, ...
                                           staticFriction, torsionalFriction, ...
                                           state, footControl) 
                                       
currentPosition = state(1:3);

xCop = footControl(1);
yCop = footControl(2);
u = footControl(3);
foot_cop = [xCop; yCop; 0];

forceDividedByMassAndU = (currentPosition - (footLocation + foot_cop)); %Being the mass and u positive quantities, while the upperbound is 0, they don't play a role

frictionBounds = [0;
                  0;
                  0];
              
A = [-yCop xCop 0];
B = [0 0 torsionalFriction];
          
friction_value = [[1 1 -(staticFriction^2)] * (forceDividedByMassAndU).^2;
                  (A-B) * forceDividedByMassAndU;
                  (-A-B) * forceDividedByMassAndU];
                 
                 
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
                 
leg_length_value = (state(1:3) - footLocation)' * (state(1:3) - footLocation);
                 
constraints = [friction_value; controlLimitValue; leg_length_value];
              
G = casadi.Function(name, {state, footControl, footLocation}, {constraints});

upperBounds = [frictionBounds;
               controlLimitsVector;
               legLength^2];                           
                                                                           
end