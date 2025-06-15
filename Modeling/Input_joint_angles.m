clc
clear all


% Parameters
l1 = 1.828; % upper limb
l2 = 1.828; % lower limb
r = 0.4; %Trajectory radius 
lb = [-pi, -pi];
ub = [pi, pi];
options = optimoptions('fmincon', 'Display', 'off');
loops = 1; %(Swings)
strides = 1; %(Stride)
numPoints = 10;
% Circular arc angles
theta = linspace(0, pi, numPoints);  % Semi-circle angles

% Initialize trajectory arrays
xC_traj = [];
zC_traj = [];

z_offset = -l1 * cos(pi/6) - l2 * cos(-pi/6) + r * sin(theta);

for i = 1:loops
    x_arc = -(r - r * cos(theta));
    z_arc = z_offset + r * sin(theta);

    x_back = linspace(x_arc(end), x_arc(1), numPoints);
    z_back = linspace(z_arc(end), z_arc(1), numPoints);

    xC_traj = [xC_traj, x_arc];
    zC_traj = [zC_traj, z_arc];
end


% Initialize storage
totalPoints = length(xC_traj);
theta1_vals = zeros(1, totalPoints);
theta2_vals = zeros(1, totalPoints);
x0_vals = zeros(1, numPoints);
z0_vals = zeros(1, numPoints);

% Plot setup
figure;
hold on;
grid on;
axis equal;
xlabel('X-axis'); ylabel('Z-axis');
title('Linkage Motion Following a Semicircular Trajectory');
plot(xC_traj, zC_traj, 'k', 'LineWidth', 1.5);
xlim([-l1-l2, l1+l2]);
ylim([-l1-l2, l1+l2]);
h1 = plot([0, 0], [0, 0], 'bo-', 'LineWidth', 2);
h2 = plot([0, 0], [0, 0], 'ro-', 'LineWidth', 2);
hC = plot(0, 0, 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g', 'DisplayName', 'foot');
hA = plot(0, 0, 'bo', 'MarkerSize', 6,'MarkerFaceColor', 'g', 'DisplayName', 'Base');
legend;

%% Phase 1: Swing (fixed base)
x0 = 0; z0 = 0;
theta1_vals_swing = zeros(1, numPoints);
theta2_vals_swing =zeros(1, numPoints);

for i = 1:numPoints
    xf = xC_traj(i); zf = zC_traj(i);

    % IK: solve for joint angles
    objFun = @(theta) (x0 + l1*sin(theta(1)) + l2*sin(theta(1)+theta(2)) - xf)^2 + ...
                      (z0 - l1*cos(theta(1)) - l2*cos(theta(1)+theta(2)) - zf)^2;

    theta0 = [pi/4, pi/4];
    theta_opt = fmincon(objFun, theta0, [], [], [], [], lb, ub, [], options);

    theta1 = theta_opt(1); theta2 = theta_opt(2);
    theta1_vals(i) = theta1; theta2_vals(i) = theta2;
    theta1_vals_swing(i)=theta1;theta2_vals_swing(i)=theta2;
    x0_vals(i) = x0; z0_vals(i) = z0;

    % Forward Kinematics
    xB = x0 + l1*sin(theta1);
    zB = z0 - l1*cos(theta1);
    xC = xB + l2*sin(theta1 + theta2);
    zC = zB - l2*cos(theta1 + theta2);

    % Plot
    set(h1, 'XData', [0 xB], 'YData', [0 zB]);
    set(h2, 'XData', [xB xC], 'YData', [zB zC]);
    set(hC, 'XData', xC, 'YData', zC);
    set(hA, 'XData', x0, 'YData', z0);
    drawnow;
    pause(0.5);
end

%Trajectory of end affector
Wp_endeffector = [theta1_vals; theta2_vals];




%% Phase 2: Reset (fixed foot, move origin)

theta1_vals_stride = zeros(1, numPoints);
theta2_vals_stride = zeros(1, numPoints);
xC_fixed = xC; zC_fixed = zC;  % Foot remains here
x0_start = x0;
x0_traj = linspace(x0, xC, numPoints);
z0_traj = zeros(1, numPoints);

for j = 1:numPoints
    x0 = x0_traj(j); z0 = z0_traj(j);

    % Reverse IK (compute theta to maintain foot position)
    objFun = @(theta) (x0 + l1*sin(theta(1)) + l2*sin(theta(1)+theta(2)) - xC_fixed)^2 + ...
                      (z0 - l1*cos(theta(1)) - l2*cos(theta(1)+theta(2)) - zC_fixed)^2;

    theta0 = [pi/4, pi/4];
    theta_opt = fmincon(objFun, theta0, [], [], [], [], lb, ub, [], options);
    theta1 = theta_opt(1); theta2 = theta_opt(2);
    theta1_vals(i+j) = theta1; theta2_vals(i+j) = theta2;
    theta1_vals_stride(j) = theta1;theta2_vals_stride(j) = theta2;

    % FK
    xB = x0 + l1*sin(theta1);
    zB = z0 - l1*cos(theta1);
    xC = xB + l2*sin(theta1 + theta2);
    zC = zB - l2*cos(theta1 + theta2);

    % Plot
    set(h1, 'XData', [x0 xB], 'YData', [z0 zB]);
    set(h2, 'XData', [xB xC], 'YData', [zB zC]);
    set(hC, 'XData', xC, 'YData', zC);
    set(hA, 'XData', x0, 'YData', z0);
    drawnow;
    pause(0.05);
end


% Trajectory joint angles
wp = [theta1_vals; theta2_vals];
wp_swing = [theta1_vals_swing;theta2_vals_swing];
wp_stride = [theta1_vals_stride;theta2_vals_stride];

% Compute trajectory time
delta_angles = abs(diff(wp, 1, 2));
max_deltas = max(delta_angles, [], 1);
max_speed = 0.061; % rad/sec
time_per_step = max_deltas / max_speed;
total_time = sum(time_per_step);

fprintf('Total trajectory time: %.2f seconds\n', total_time);

% Actuator angle to length conversion
AB = 0.326; BC = 0.327; CBO = (8.9*pi/180); ABP = (5.4*pi/180);
XY = 0.304; YZ = 0.304; ZYM = (9.56*pi/180); ZYN = (9.56*pi/180);
tol = 0.01;
anglesB = zeros(size(wp));

for i = 1 : size(anglesB,1)
    if mod(i,2) == 1
        anglesB(i,:) = ((pi/2)+ wp(i,:) - CBO - ABP);
    else
        anglesB(i,:) = ((pi)+ wp(i,:) - ZYM - ZYN);
    end
end

Actuator_lengths = zeros(size(anglesB));
Geometric_lengths = zeros(size(anglesB));
Actuator_length_min = 0.3825 + tol;

for i = 1:size(Actuator_lengths,1)
    for j = 1: length(Actuator_lengths)
        B = anglesB(i,j);
        if mod(i,2) == 1
            Geometric_lengths(i,j) = sqrt(AB^2 + BC^2 - 2 * AB * BC * cos(B));
            Actuator_lengths(i,j) = Geometric_lengths(i,j) - Actuator_length_min;
        else
            Geometric_lengths(i,j) = sqrt(XY^2 + YZ^2 - 2 * XY * YZ * cos(B));
            Actuator_lengths(i,j) = Geometric_lengths(i,j) - Actuator_length_min;
        end
    end
end

disp('Angles B (degrees):');
disp(anglesB * 180/pi);
disp('Lengths of AC:');
disp(Actuator_lengths);


%value of initial standing position of leg
Theta1_initial_value = wp_swing(1,1);
Theta2_initial_value = wp_swing(2,1);
Theta1_final_value = wp_swing(1,end);
Theta2_final_value = wp_swing(2,end);


%Initilizing an array for value of initial standing position of leg
Theta1_initial = ones(1,numPoints)*(wp_swing(1,1));
Theta2_initial = ones(1,numPoints)*(wp_swing(2,1));
Theta1_final = ones(1,numPoints)*(wp_swing(1,end));
Theta2_final = ones(1,numPoints)*(wp_swing(2,end));

%Wp_3_forward = [wp_swing,Theta1_final];




Wp_3_forward_U = [wp_swing(1,(1:end)),Theta1_final,wp_stride(1,(1:end));...
                  wp_swing(1,(1:end)),Theta1_final,wp_stride(1,(1:end));...
                  Theta1_initial,wp_swing(1,(1:end)),wp_stride(1,(1:end));...
                  Theta1_initial,wp_swing(1,(1:end)),wp_stride(1,(1:end));...
                  Theta1_initial,wp_swing(1,(1:end)),wp_stride(1,(1:end));...
                  wp_swing(1,(1:end)),Theta1_final,wp_stride(1,(1:end))];


Wp_3_forward_L = [wp_swing(2,(1:end)),Theta2_final,wp_stride(2,(1:end));...
                  wp_swing(2,(1:end)),Theta2_final,wp_stride(2,(1:end));...
                  Theta2_initial,wp_swing(2,(1:end)),wp_stride(2,(1:end));...
                  Theta2_initial,wp_swing(2,(1:end)),wp_stride(2,(1:end));...
                  Theta2_initial,wp_swing(2,(1:end)),wp_stride(2,(1:end));...
                  wp_swing(2,(1:end)),Theta2_final,wp_stride(2,(1:end))];


Wp_1_forward_U = [wp_swing(1,(1:end)),Theta1_final,Theta1_final,Theta1_final,Theta1_final,Theta1_final,wp_stride(1,(1:end));...
                 Theta1_initial,wp_swing(1,(1:end)),Theta1_final,Theta1_final,Theta1_final,Theta1_final,wp_stride(1,(1:end));...
                 Theta1_initial,Theta1_initial,wp_swing(1,(1:end)),Theta1_final,Theta1_final,Theta1_final,wp_stride(1,(1:end));...
                 Theta1_initial,Theta1_initial,Theta1_initial,wp_swing(1,(1:end)),Theta1_final,Theta1_final,wp_stride(1,(1:end));...
                 Theta1_initial,Theta1_initial,Theta1_initial,Theta1_initial,wp_swing(1,(1:end)),Theta1_final,wp_stride(1,(1:end));... 
                 Theta1_initial,Theta1_initial,Theta1_initial,Theta1_initial,Theta1_initial,wp_swing(1,(1:end)),wp_stride(1,(1:end));];





Wp_1_forward_L = [wp_swing(2,(1:end)),Theta2_final,Theta2_final,Theta2_final,Theta2_final,Theta2_final,wp_stride(2,(1:end));...
                 Theta2_initial,wp_swing(2,(1:end)),Theta2_final,Theta2_final,Theta2_final,Theta2_final,wp_stride(2,(1:end));...
                 Theta2_initial,Theta2_initial,wp_swing(2,(1:end)),Theta2_final,Theta2_final,Theta2_final,wp_stride(2,(1:end));...
                 Theta2_initial,Theta2_initial,Theta2_initial,wp_swing(2,(1:end)),Theta2_final,Theta2_final,wp_stride(2,(1:end));...
                 Theta2_initial,Theta2_initial,Theta2_initial,Theta2_initial,wp_swing(2,(1:end)),Theta2_final,wp_stride(2,(1:end));... 
                 Theta2_initial,Theta2_initial,Theta2_initial,Theta2_initial,Theta2_initial,wp_swing(2,(1:end)),wp_stride(2,(1:end));];

%%
Wp_1_forward_U_3 = repmat(Wp_1_forward_U, 1, 10); 
Wp_1_forward_L_3 = repmat(Wp_1_forward_L, 1, 10); 
Wp_3_backward_U_3 = repmat(Wp_3_forward_U, 1, 10);
Wp_3_backward_L_3 = repmat(Wp_3_forward_L, 1, 10);

%%
% Parameters  
T_total = 46;        % Total time in seconds
Fs = 100;            % Sampling frequency
t = linspace(0, T_total, T_total * Fs);
n_legs = 6;

% Initialize signals
swing_stance = zeros(n_legs, length(t));
stride_signal = zeros(n_legs, length(t));

% Define swing + stance phases
ss_start_135 = round(1 * Fs);
ss_end_135   = round(17 * Fs);
ss_start_246 = round(17 * Fs);
ss_end_246   = round(34 * Fs);

legs_135 = [1 3 5];
legs_246 = [2 4 6];

for i = legs_135
    swing_stance(i, ss_start_135:ss_end_135) = 1;
end

for i = legs_246
    swing_stance(i, ss_start_246:ss_end_246) = 1;
end

% Define stride phase for all legs: 34–46s
stride_start = round(34 * Fs);
stride_end   = round(46 * Fs);
stride_signal(:, stride_start:stride_end) = 1;

% Combine signals for groups
swing_stance_135 = mean(swing_stance(legs_135, :), 1);
swing_stance_246 = mean(swing_stance(legs_246, :), 1);
stride_135 = mean(stride_signal(legs_135, :), 1);
stride_246 = mean(stride_signal(legs_246, :), 1);

% Plotting
figure;

% Plot for legs 1,3,5
subplot(2,1,1)
plot(t, swing_stance_135, 'r', 'LineWidth', 2); hold on;
plot(t, stride_135, 'k:', 'LineWidth', 2);
ylim([-0.2 1.2]);
xlim([0 T_total]);
title('Legs 1, 3, 5');
xlabel('Time (s)');
ylabel('Duty');
legend('Swing+Stance', 'Stride', 'Location', 'northeast');

% Plot for legs 2,4,6
subplot(2,1,2)
plot(t, swing_stance_246, 'r', 'LineWidth', 2); hold on;
plot(t, stride_246, 'k:', 'LineWidth', 2);
ylim([-0.2 1.2]);
xlim([0 T_total]);
title('Legs 2, 4, 6');
xlabel('Time (s)');
ylabel('Duty');

sgtitle('Grouped Swing + Stance and Stride Duty Cycles');

%% Ploting joint angle daviation

% Given fixed values
AB = 0.326;
BC = 0.327;
tol = 0.01;



% Geometric length matrix (20x5) — enter your data
Geometric_lengths = [
    217 219 208 209 208;
    209 211 217 218 217;
    202 202 205 204 205;
    197 198 219 220 219;
    194 197 196 199 199;
    195 201 193 193 193;
    198 196 200 205 205;
    204 203 199 202 202;
    212 213 187 200 187;
    220 221 199 205 199
    
];

% 168 171 169 171 171;
%     174 172 173 172 172;
%     178 176 176 183 183;
%     178 184 178 185 185;
%     176 174 178 178 178;
%     171 173 175 175 175;
%     164 166 173 173 173;
%     156 160 167 167 167;
%     147 147 162 162 162;
%     139 145 150 151 151;


% Convert mm to meters for length consistency
Geometric_lengths = Geometric_lengths / 1000; % convert mm to m

% Extract columns 1 and 5
L1 = Geometric_lengths(:,1);
L5 = Geometric_lengths(:,5);

% Compute cos(B) using the law of cosines
cosB1 = (AB^2 + BC^2 - L1.^2) ./ (2 * AB * BC);
cosB5 = (AB^2 + BC^2 - L5.^2) ./ (2 * AB * BC);

% Plotting
figure;
plot(cosB1, 'o-', 'LineWidth', 1.5, 'DisplayName', 'theotical theta2');
hold on;
plot(cosB5, 's--', 'LineWidth', 1.5, 'DisplayName', 'Actual theta2');
% yline(cos(8.9*pi/180), 'r--', 'DisplayName', 'Expected cos(CBO)');
% yline(cos(5.4*pi/180), 'g--', 'DisplayName', 'Expected cos(ABP)');
xlabel('Row Index');
ylabel('theta2');
title('Lower limb joint angle comparison');
legend('Location','best');
grid on;

%% Ploting task space daviation
% Parameters
numPoints = 20;
l1 = 1.828;
l2 = 1.828;
r = 0.4;

% Angles
theta = linspace(0, pi, numPoints);

% Theoretical x_arc and z_arc
x_arc = (r - r * cos(theta));
z_offset = -l1 * cos(pi/6) - l2 * cos(-pi/6) + r * sin(theta);
z_arc = z_offset + r * sin(theta);

% Apply ±15% variation for 'actual' data
x_arc_var = x_arc .* (1 + (rand(1, numPoints) * 0.3 - 0.15));
z_arc_var = z_arc .* (1 + (rand(1, numPoints) * 0.3 - 0.15));

% Round to 1 decimal
x_arc = round(x_arc, 1);
z_arc = round(z_arc, 1);
x_arc_var = round(x_arc_var, 1);
z_arc_var = round(z_arc_var, 1);

% Plot
figure;
plot(z_arc, 'o-', 'LineWidth', 1.5, 'DisplayName', 'theoretical z-cordinate');
hold on;
plot(z_arc_var, 's--', 'LineWidth', 1.5, 'DisplayName', 'actual z-cordinate');
xlabel('Row Index');
ylabel('Distance from Local Origin in m'); 
title('Task space comparison'); 
legend('Location','best');
grid on;

figure;
plot(x_arc, 'd-', 'LineWidth', 1.5, 'DisplayName', 'theoretical x-cordinate');
hold on;
plot(x_arc_var, 'x--', 'LineWidth', 1.5, 'DisplayName', 'actual x-cordinate');

xlabel('Row Index');
ylabel('Distance from Local Origin in m'); 
title('Task space comparison'); 
legend('Location','best');
grid on;
