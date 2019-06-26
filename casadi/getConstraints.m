function [G, upperBounds] = getConstraints(name, footLocation, ...
                                           copLimits, legLength, ...
                                           staticFriction, torsionalFriction, ...
                                           state, footControl) 
                                       
px = state(1);
py = state(2);
pz = state(3);
pxFoot = footLocation(1);
pyFoot = footLocation(2);
pzFoot = footLocation(3);

xCop = footControl(1);
yCop = footControl(2);
u = footControl(3);
foot_cop = [xCop; yCop];

frictionBounds = [staticFriction^2;
                  torsionalFriction;
                  torsionalFriction];
    
torsional_multiplier = [(py - pyFoot)/(pz - pzFoot), ...
                       -(px - pxFoot)/(pz - pzFoot)];
              
              
friction_value = [((px - pxFoot - xCop)/(pz - pzFoot))^2 + ...
                 ((py - pyFoot - yCop)/(pz - pzFoot))^2;
                 torsional_multiplier * foot_cop;
                 -torsional_multiplier * foot_cop];
                 
                 
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