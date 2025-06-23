clc; clear; close all;

%% PARAMETERS
L_total = 0.56;
M_total = 2.37;
m1_ratio = 0.48;
m2_ratio = 0.52;

%% TIME SETUP
dt = 0.001;
T = 10;
time = 0:dt:T;

%% TRAJECTORY: Circle Path
r = 0.15;
cx = 0.150; cy = 0.4;
xd = cx + r*cos(2*pi*0.1*time);
yd = cy + r*sin(2*pi*0.1*time);

%% OPTIMIZATION FOR BEST L1 BASED ON MANIPULABILITY
l1_values = linspace(0.1, 0.5, 100);
best_score = -Inf;
best_l1 = 0;
all_scores = zeros(size(l1_values));

for k = 1:length(l1_values)
    l1 = l1_values(k);
    l2 = L_total - l1;
    m1 = m1_ratio * M_total;
    m2 = m2_ratio * M_total;

    th1 = zeros(size(time));
    th2 = zeros(size(time));
    manip_index = zeros(size(time));

    for i = 1:length(time)
        x = xd(i); y = yd(i);
        D = (x^2 + y^2 - l1^2 - l2^2)/(2*l1*l2);
        if abs(D) > 1
            manip_index(i) = 0;
            continue;
        end
        th2(i) = atan2(-sqrt(1 - D^2), D);
        th1(i) = atan2(y, x) - atan2(l2*sin(th2(i)), l1 + l2*cos(th2(i)));

        J = [-l1*sin(th1(i)) - l2*sin(th1(i)+th2(i)), -l2*sin(th1(i)+th2(i));
              l1*cos(th1(i)) + l2*cos(th1(i)+th2(i)),  l2*cos(th1(i)+th2(i))];
        manip_index(i) = sqrt(det(J*J'));
    end

    avg_manip = mean(manip_index);
    all_scores(k) = avg_manip;

    if avg_manip > best_score
        best_score = avg_manip;
        best_l1 = l1;
    end
end

fprintf('Best femur length l1 = %.3f m\n', best_l1);

% Use optimal l1 for simulation
l1 = best_l1;
l2 = L_total - l1;
m1 = m1_ratio * M_total;
m2 = m2_ratio * M_total;

%% INVERSE KINEMATICS
th1 = zeros(size(time));
th2 = zeros(size(time));

for i = 1:length(time)
    x = xd(i); y = yd(i);
    D = (x^2 + y^2 - l1^2 - l2^2)/(2*l1*l2);
    D = min(max(D, -1), 1);
    th2(i) = atan2(-sqrt(1 - D^2), D);
    th1(i) = atan2(y, x) - atan2(l2*sin(th2(i)), l1 + l2*cos(th2(i)));
end

%% FORWARD KINEMATICS
x0 = zeros(size(time)); y0 = zeros(size(time));
x1 = l1*cos(th1); y1 = l1*sin(th1);
x2 = x1 + l2.*cos(th1 + th2);
y2 = y1 + l2.*sin(th1 + th2);

%% REALISTIC ANIMATION
figure;
hold on; axis equal; grid on;
xlim([-0.4 0.4]); ylim([-0.1 0.6]);
xlabel('X (m)'); ylabel('Y (m)');
title('Realistic 2-DOF Robot Leg Animation');

% Ground line
plot([-0.5 0.5], [0 0], 'k--');

% Trajectory path
plot(xd, yd, 'g--', 'LineWidth', 1);

% Handles
femur_line = plot([0,0],[0,0],'r','LineWidth',3);
shank_line = plot([0,0],[0,0],'b','LineWidth',3);
hip_joint = plot(0, 0, 'ko', 'MarkerFaceColor', 'k');
knee_joint = plot(0, 0, 'ko', 'MarkerFaceColor', 'r');
foot_joint = plot(0, 0, 'ko', 'MarkerFaceColor', 'b');
foot_path = plot(x2(1), y2(1), 'm.');

% Animation loop
for i = 1:10:length(time)
    set(femur_line, 'XData', [0, x1(i)], 'YData', [0, y1(i)]);
    set(shank_line, 'XData', [x1(i), x2(i)], 'YData', [y1(i), y2(i)]);
    set(knee_joint, 'XData', x1(i), 'YData', y1(i));
    set(foot_joint, 'XData', x2(i), 'YData', y2(i));
    set(foot_path, 'XData', x2(1:i), 'YData', y2(1:i));
    pause(0.01);
end

%% Plot optimization result
figure;
plot(l1_values, all_scores, 'LineWidth', 2);
xlabel('Femur Length l1 (m)');
ylabel('Average Manipulability Index');
title('Optimization of Femur Length l1');
grid on;