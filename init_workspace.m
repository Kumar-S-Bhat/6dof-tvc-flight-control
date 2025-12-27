%% Initialize 6DOF Aircraft Simulation Workspace
% Sets up all aircraft parameters, initial conditions, and constants

clear; clc; close all;

fprintf('========================================\n');
fprintf('6DOF THRUST VECTORING AIRCRAFT SIMULATION\n');
fprintf('========================================\n\n');

%% Aircraft Parameters (F-16 class fighter)
aircraft.mass = 9300;                    % kg
aircraft.Ixx = 12875;                    % kg*m^2 (roll inertia)
aircraft.Iyy = 55814;                    % kg*m^2 (pitch inertia)
aircraft.Izz = 63100;                    % kg*m^2 (yaw inertia)
aircraft.Ixz = 1331;                     % kg*m^2 (product of inertia)

aircraft.S = 27.87;                      % m^2 (wing area)
aircraft.b = 9.144;                      % m (wing span)
aircraft.c = 3.45;                       % m (mean aerodynamic chord)

% Inertia matrix (will be used by 6DOF block)
aircraft.I = [aircraft.Ixx 0 -aircraft.Ixz; 
              0 aircraft.Iyy 0; 
              -aircraft.Ixz 0 aircraft.Izz];

%% Control Surface Limits
control.elevator_max = deg2rad(25);      % rad
control.aileron_max = deg2rad(21.5);     % rad
control.rudder_max = deg2rad(30);        % rad
control.nozzle_pitch_max = deg2rad(20);  % rad (TVC pitch)
% control.nozzle_yaw_max = deg2rad(0);    % rad (TVC yaw)

%% Engine Parameters
engine.thrust_max = 130000;              % N (max static thrust at sea level)
engine.nozzle_arm = 6.0;                 % m (moment arm from CG)  

%% Atmospheric Constants
atmos.rho_sl = 1.225;                    % kg/m^3
atmos.g = 9.81;                          % m/s^2
atmos.R = 287.05;                        % J/(kg*K) - gas constant

%% Loading aerodynamic coefficients
fprintf('Loading Aerodynamic Data')
load_aero_data();

%% Initial Conditions (Trimmed straight and level flight)
IC.airspeed = 150;                       % m/s
IC.altitude = 3000;                      % m
IC.gamma = 0;                            % rad (flight path angle)

%% Find Trim (6DOF)
fprintf('Finding trim condition...\n');
trim_result = trim_solver(IC.airspeed, IC.altitude, IC.gamma);
 
if ~trim_result.success
    warning('Trim solver did not converge well. Check residuals.');
end

% Update initial conditions from trim
% State vector: [u v w p q r phi theta psi x y z]
IC.u = trim_result.u;                    % m/s
IC.v = trim_result.v;                    % m/s (body y velocity)
IC.w = trim_result.w;                    % m/s (body z velocity)
IC.p = 0;                                % rad/s (roll rate)
IC.q = 0;                                % rad/s (pitch rate)
IC.r = 0;                                % rad/s (yaw rate)
IC.phi = trim_result.phi;                % rad (roll angle)
IC.theta = trim_result.theta;            % rad (pitch angle - small for trim)
IC.psi = 0;                              % rad (yaw angle)
IC.x = 0;                                % m (north position)
IC.y = 0;                                % m (east position)
IC.z = -IC.altitude;                     % m (down position, NED frame)

% Trim control positions (from trim analysis)
IC.throttle = trim_result.throttle;
IC.elevator = trim_result.delta_e;       % rad
IC.aileron = trim_result.delta_a;        % rad
IC.rudder = trim_result.delta_r;         % rad
IC.nozzle_pitch = trim_result.delta_p;   % rad

%% Simulation Parameters
sim_params.duration = 50;                % s (simulation time)

%% Controller Gains
% Pitch rate controller
controller.pitch.Kp = 75000;
controller.pitch.Ki = 13000;
controller.pitch.Kd = 0;

% Roll rate controller
controller.roll.Kp = 50000;
controller.roll.Ki = 5000;
controller.roll.Kd = 5000;

% Yaw rate controller
controller.yaw.Kp = 40000;
controller.yaw.Ki = 3000;
controller.yaw.Kd = 8000;

% Outer loop (angle commands to rate commands)
controller.alpha.Kp = 3.0;
controller.alpha.Ki = 20.0;
controller.alpha.Kd = 0.056;

controller.beta.Kp = 2.0;
controller.beta.Ki = 5.0;
controller.beta.Kd = 0.1;

%% Display Summary
fprintf('\n✓ Workspace initialized successfully\n');
fprintf('========================================\n');
fprintf('Aircraft Configuration:\n');
fprintf('  Mass: %.0f kg\n', aircraft.mass);
fprintf('  Wing Area: %.2f m²\n', aircraft.S);
fprintf('  Max Thrust: %.0f kN\n', engine.thrust_max/1000);
fprintf('\nInitial Conditions:\n');
fprintf('  Airspeed: %.0f m/s\n', IC.airspeed);
fprintf('  Altitude: %.0f m\n', IC.altitude);
fprintf('  Pitch: %.2f°\n', rad2deg(IC.theta));
fprintf('\nSimulation Settings:\n');
fprintf('  Duration: %.0f s\n', sim_params.duration);
fprintf('========================================\n');