%% Analyze Simulation Results

if ~exist('out', 'var')
    error('No simulation data found. Run the Simulink model first.');
end

fprintf('========================================\n');
fprintf('POST-SIMULATION ANALYSIS\n');
fprintf('========================================\n\n');

t = out.tout;

% Extract states
u = out.states(:,1);
v = out.states(:,2);
w = out.states(:,3);
p = out.states(:,4);
q = out.states(:,5);
r = out.states(:,6);
phi = out.states(:,7);
theta = out.states(:,8);
psi = out.states(:,9);
x = out.states(:,10);
y = out.states(:,11);
z = out.states(:,12);

V = sqrt(u.^2 + v.^2 + w.^2);           % Total airspeed
alpha = atan2(w, u);                     % Angle of attack
beta = asin(v ./ V);                     % Sideslip angle
altitude = -z;                           % Altitude (NED convention)
gamma = asin(-w ./ V);                   % Flight path angle

%% Performance Metrics
fprintf('Performance Summary:\n');
fprintf('  Final altitude: %.0f m (Δh = %.0f m)\n', altitude(end), altitude(end) - altitude(1));
fprintf('  Final airspeed: %.1f m/s\n', V(end));
fprintf('  Max pitch rate: %.1f deg/s\n', max(abs(rad2deg(q))));
fprintf('  Max roll rate: %.1f deg/s\n', max(abs(rad2deg(p))));
fprintf('  Max alpha: %.1f deg\n', max(rad2deg(alpha)));
fprintf('  Max beta: %.1f deg\n', max(abs(rad2deg(beta))));

%% Create plots
figure('Name', '6DOF Aircraft Simulation Results', 'Position', [100 100 1400 900]);

% Subplot 1: Velocities
subplot(3,3,1);
plot(t, u, 'b-', 'LineWidth', 1.5); hold on;
plot(t, v, 'r-', 'LineWidth', 1.5);
plot(t, w, 'g-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Body Frame Velocities');
legend('u (forward)', 'v (side)', 'w (down)', 'Location', 'best');

% Subplot 2: Angular Rates
subplot(3,3,2);
plot(t, rad2deg(p), 'b-', 'LineWidth', 1.5); hold on;
plot(t, rad2deg(q), 'r-', 'LineWidth', 1.5);
plot(t, rad2deg(r), 'g-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Angular Rate (deg/s)');
title('Body Frame Angular Rates');
legend('p (roll)', 'q (pitch)', 'r (yaw)', 'Location', 'best');

% Subplot 3: Euler Angles
subplot(3,3,3);
plot(t, rad2deg(phi), 'b-', 'LineWidth', 1.5); hold on;
plot(t, rad2deg(theta), 'r-', 'LineWidth', 1.5);
plot(t, rad2deg(psi), 'g-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Angle (deg)');
title('Euler Angles');
legend('\phi (roll)', '\theta (pitch)', '\psi (yaw)', 'Location', 'best');

% Subplot 4: Airspeed & Altitude
subplot(3,3,4);
yyaxis left;
plot(t, V, 'b-', 'LineWidth', 1.5);
ylabel('Airspeed (m/s)');
yyaxis right;
plot(t, altitude, 'r-', 'LineWidth', 1.5);
ylabel('Altitude (m)');
xlabel('Time (s)');
title('Airspeed and Altitude');
grid on;

% Subplot 5: Alpha & Beta
subplot(3,3,5);
plot(t, rad2deg(alpha), 'b-', 'LineWidth', 1.5); hold on;
plot(t, rad2deg(beta), 'r-', 'LineWidth', 1.5);
yline(15, 'k--', 'Stall');
grid on;
xlabel('Time (s)');
ylabel('Angle (deg)');
title('Angle of Attack and Sideslip');
legend('\alpha', '\beta', 'Location', 'best');

% Subplot 6: 3D Flight Path
subplot(3,3,6);
plot3(x/1000, y/1000, altitude, 'b-', 'LineWidth', 2);
hold on;
plot3(x(1)/1000, y(1)/1000, altitude(1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot3(x(end)/1000, y(end)/1000, altitude(end), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
grid on;
xlabel('North (km)');
ylabel('East (km)');
zlabel('Altitude (m)');
title('3D Flight Path');
legend('Trajectory', 'Start', 'End', 'Location', 'best');
view(45, 30);

% Subplot 7: Control Deflections (if available)
if isfield(out, 'controls')
    subplot(3,3,7);
    plot(t, rad2deg(out.controls(:,1)), 'b-', 'LineWidth', 1.5); hold on;
    plot(t, rad2deg(out.controls(:,2)), 'r-', 'LineWidth', 1.5);
    plot(t, rad2deg(out.controls(:,3)), 'g-', 'LineWidth', 1.5);
    yline(25, 'k--');
    yline(-25, 'k--');
    grid on;
    xlabel('Time (s)');
    ylabel('Deflection (deg)');
    title('Control Surface Positions');
    legend('Elevator', 'Aileron', 'Rudder', 'Location', 'best');
end

% Subplot 8: Energy
subplot(3,3,8);
KE = 0.5 * aircraft.mass * V.^2 / 1e6;  % Kinetic energy (MJ)
PE = aircraft.mass * atmos.g * altitude / 1e6;  % Potential energy (MJ)
TE = KE + PE;  % Total energy (MJ)
plot(t, KE, 'b-', 'LineWidth', 1.5); hold on;
plot(t, PE, 'r-', 'LineWidth', 1.5);
plot(t, TE, 'k-', 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Energy (MJ)');
title('Energy Analysis');
legend('Kinetic', 'Potential', 'Total', 'Location', 'best');

% Subplot 9: Trajectory (Top View)
subplot(3,3,9);
plot(x/1000, y/1000, 'b-', 'LineWidth', 2); hold on;
plot(x(1)/1000, y(1)/1000, 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot(x(end)/1000, y(end)/1000, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
grid on;
axis equal;
xlabel('North (km)');
ylabel('East (km)');
title('Ground Track (Top View)');
legend('Path', 'Start', 'End', 'Location', 'best');

% Save figure
% saveas(gcf, '../results/simulation_results.png');
% fprintf('\n✓ Analysis complete. Figure saved to results/\n');