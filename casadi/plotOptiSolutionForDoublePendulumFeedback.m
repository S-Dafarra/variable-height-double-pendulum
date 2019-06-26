function plotOptiSolutionForDoublePendulumFeedback(xsol, usol, a_sol, t_sol, k_sol, x_des_sol)
time = 0.0;
numberOfPhases = length(t_sol);
phase_length = size(usol,2)/numberOfPhases;
for i = 1 : numberOfPhases
    time = [time, linspace(time(end) + t_sol(i)/phase_length, time(end) + t_sol(i), phase_length)];
end

figure
ax = axes;
plot(time, xsol(1:3,:));
t = 0;
for i = 1 : length(t_sol)
    t = t + t_sol(i);
    line([t t],get(ax,'YLim'),'Color','k', 'LineStyle','--')
end
title("CoM Position")
legend(["x", "y", "z"]);
ylabel("[m]");
xlabel("t [s]");

figure
ax = axes;
plot(time, xsol(4:6,:));
t = 0;
for i = 1 : length(t_sol)
    t = t + t_sol(i);
    line([t t],get(ax,'YLim'),'Color','k', 'LineStyle','--')
end
title("CoM Velocity")
legend(["x", "y", "z"]);
ylabel("[m/s]");
xlabel("t [s]");

figure
ax = axes;
plot(time(2:end), a_sol(1:3,:));
t = 0;
for i = 1 : length(t_sol)
    t = t + t_sol(i);
    line([t t],get(ax,'YLim'),'Color','k', 'LineStyle','--')
end
title("CoM Acceleration (without gravity)")
legend(["x", "y", "z"]);
ylabel("[m/s^2]");
xlabel("t [s]");

figure
ax = axes;
t = 0;
plot(time(2:end), usol(3,:), time(2:end), usol(6,:));
for i = 1 : length(t_sol)
    t = t + t_sol(i);
    line([t t],get(ax,'YLim'),'Color','k', 'LineStyle','--')
end
legend(["uLeft", "uRight"]);
title("u")
ylabel("[1/s^2]");
xlabel("t [s]");

figure
ax = axes;
t = 0;
plot(time(2:end), usol(1:2,:));
for i = 1 : length(t_sol)
    t = t + t_sol(i);
    line([t t],get(ax,'YLim'),'Color','k', 'LineStyle','--')
end
title("CoP Left")
legend(["x", "y"]);
ylabel("[m]");
xlabel("t [s]");

figure
ax = axes;
t = 0;
plot(time(2:end), usol(4:5,:));
for i = 1 : length(t_sol)
    t = t + t_sol(i);
    line([t t],get(ax,'YLim'),'Color','k', 'LineStyle','--')
end
title("CoP Right")
legend(["x", "y"]);
ylabel("[m]");
xlabel("t [s]");

k_timed = zeros(6, length(time) - 1);
ff_timed = zeros(6, length(time) - 1);

for i = 1 : length(t_sol)
    for j = (i-1) * phase_length + 1 : i * phase_length
        k_timed(:, j) = k_sol(:, i);
        ff_timed(:, j) = x_des_sol(:, i);
    end
end

figure
ax = axes;
plot(time(2:end), k_timed(1:3,:));
t = 0;
for i = 1 : length(t_sol)
    t = t + t_sol(i);
    line([t t],get(ax,'YLim'),'Color','k', 'LineStyle','--')
end
title("Position gains")
legend(["x", "y", "z"]);
ylabel("[1/s^2]");
xlabel("t [s]");

figure
ax = axes;
plot(time(2:end), k_timed(4:6,:));
t = 0;
for i = 1 : length(t_sol)
    t = t + t_sol(i);
    line([t t],get(ax,'YLim'),'Color','k', 'LineStyle','--')
end
title("Velocity gains")
legend(["x", "y", "z"]);
ylabel("[1/s]");
xlabel("t [s]");

figure
ax = axes;
plot(time(2:end), ff_timed(1:3,:));
t = 0;
for i = 1 : length(t_sol)
    t = t + t_sol(i);
    line([t t],get(ax,'YLim'),'Color','k', 'LineStyle','--')
end
title("Optimized Desired Position")
legend(["x", "y", "z"]);
ylabel("[m]");
xlabel("t [s]");

figure
ax = axes;
plot(time(2:end), ff_timed(4:6,:));
t = 0;
for i = 1 : length(t_sol)
    t = t + t_sol(i);
    line([t t],get(ax,'YLim'),'Color','k', 'LineStyle','--')
end
title("Optimized Desired Velocity")
legend(["x", "y", "z"]);
ylabel("[m/s]");
xlabel("t [s]");

desiredAcceleration = zeros(3, length(time) - 1);
for i = 1 : length(time) - 1 
    gainMatrix = [diag(k_timed(1:3,i)), diag(k_timed(4:6,i))];
    desiredAcceleration(:,i) = +[0;0;9.81] - gainMatrix * (xsol(:, i) - ff_timed(:, i));
end

figure
ax = axes;
plot(time(2:end), desiredAcceleration(1:3,:));
hold on
plot(time(2:end), a_sol(1:3,:),'LineStyle','--');
t = 0;
for i = 1 : length(t_sol)
    t = t + t_sol(i);
    line([t t],get(ax,'YLim'),'Color','k', 'LineStyle','--')
end
title("Optimized Desired Acceleration")
legend(["x", "y", "z"]);
ylabel("[m/s^2]");
xlabel("t [s]");

end