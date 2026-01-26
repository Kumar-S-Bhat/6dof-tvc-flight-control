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

%% Control Surface Features
control.elevator_max = deg2rad(25);      % rad
control.aileron_max = deg2rad(21.5);     % rad
control.rudder_max = deg2rad(30);        % rad
control.nozzle_pitch_max = deg2rad(20);  % rad (TVC pitch)

% Rate Limits
control.max_elevator_rate = deg2rad(60);    % rad/s
control.max_aileron_rate  = deg2rad(80);    % rad/s
control.max_rudder_rate   = deg2rad(60);    % rad/s
control.max_throttle_rate = 0.20;           % 20%/s
control.max_nozzle_rate   = deg2rad(40);    % rad/s

% 2nd Order Actuator Dynamics - Natural Frequencies
control.wn_elevator = 2*pi*12;   % 12 Hz (~75 rad/s) - primary pitch control
control.wn_aileron  = 2*pi*15;   % 15 Hz (~94 rad/s) - fastest for roll
control.wn_rudder   = 2*pi*10;   % 10 Hz (~63 rad/s) - slower, larger surface
control.wn_throttle = 2*pi*1.5;  % 1.5 Hz (~9.4 rad/s) - engine response
control.wn_nozzle   = 2*pi*8;    % 8 Hz (~50 rad/s) - hydraulic/electric TVC

% Damping Ratios (ζ)
control.zeta_elevator = 0.7;     % slightly underdamped for quick response
control.zeta_aileron  = 0.7;     % slightly underdamped for quick response
control.zeta_rudder   = 0.8;     % more damped, stability priority
control.zeta_throttle = 1.0;     % critically damped, no overshoot desired
control.zeta_nozzle   = 0.75;    % balance between speed and stability

%% Engine Parameters
engine.thrust_max = 130000;              % N (max static thrust at sea level)
engine.nozzle_arm = 6.0;                 % m (moment arm from CG)  

%% Atmospheric Constants
atmos.rho_sl = 1.225;                    % kg/m^3
atmos.g = 9.81;                          % m/s^2
atmos.R = 287.05;                        % J/(kg*K) - gas constant

%% Simulation Parameters
sim_params.duration = 30;                % s (simulation time)

%% Loading aerodynamic coefficients
fprintf('Loading Aerodynamic Data')
load_aero_data();

%% Load Gain-Scheduled LQR Controller
fprintf('\nLoading LQR gain ...\n');
if exist('data/lqr_gain.mat', 'file')
    load('data/lqr_gain.mat', 'controller');
    fprintf('✓ LQR gain loaded \n')
else
    error('LQR gain not found! Run generate_lqr_gain() first.');
end

%% Initial Conditions (Trimmed straight and level flight)
IC.airspeed = 150;                       % m/s
IC.altitude = 3000;                      % m

IC.u=IC.airspeed;IC.v=0;IC.w=0;
IC.x=0;IC.y=0;IC.z=-IC.altitude;
IC.p=0;IC.q=0;IC.r=0;
IC.phi=0;IC.theta=0;IC.psi=0;

%% Initialize Actuator Blocks (pre-trim)
actuator_blocks = {
    'aircraft_6dof/Actuator Lag/elevator lag/Linear Second-Order Actuator'
    'aircraft_6dof/Actuator Lag/aileron lag/Linear Second-Order Actuator'
    'aircraft_6dof/Actuator Lag/rudder lag/Linear Second-Order Actuator'
    'aircraft_6dof/Actuator Lag/throttle lag/Linear Second-Order Actuator'
    'aircraft_6dof/Actuator Lag/nozzle lag/Linear Second-Order Actuator'
};

% Reset all to '0' before trim
for i = 1:length(actuator_blocks)
    set_param(actuator_blocks{i}, 'fin_act_0', '0');
end

%% Find Trim (6DOF)
% Update initial conditions from trim
fprintf('Finding trim condition...\n');
[IC, op] = trim_solver(IC.airspeed, IC.altitude);

%% Set initial condition to actuators
set_param('aircraft_6dof/Actuator Lag/elevator lag/Linear Second-Order Actuator','fin_act_0','IC.elevator');
set_param('aircraft_6dof/Actuator Lag/aileron lag/Linear Second-Order Actuator','fin_act_0', 'IC.aileron');
set_param('aircraft_6dof/Actuator Lag/rudder lag/Linear Second-Order Actuator','fin_act_0', 'IC.rudder');
set_param('aircraft_6dof/Actuator Lag/throttle lag/Linear Second-Order Actuator','fin_act_0', 'IC.throttle');
set_param('aircraft_6dof/Actuator Lag/nozzle lag/Linear Second-Order Actuator','fin_act_0', 'IC.nozzle_pitch');

%% Display Summary
fprintf('\n✓ Workspace initialized successfully\n');
fprintf('========================================\n');
fprintf('Aircraft Configuration:\n');
fprintf('  Mass: %.0f kg\n', aircraft.mass);
fprintf('  Wing Area: %.2f m²\n', aircraft.S);
fprintf('  Max Thrust: %.0f kN\n', engine.thrust_max/1000);
fprintf('\nInitial Conditions:\n');
fprintf('  Airspeed: %.0f m/s\n', IC.airspeed);
fprintf('  Altitude: %.0f m\n', -IC.z);
fprintf('  Pitch: %.2f°\n', rad2deg(IC.theta));
fprintf('\nSimulation Settings:\n');
fprintf('  Duration: %.0f s\n', sim_params.duration);
fprintf('========================================\n');

% State Vector: [u v w x y z p q r phi theta psi]
% u, v, w        : Body-frame velocities (m/s)
% x, y, z        : NED position (m, down is positive)
% p, q, r        : Body angular rates (rad/s)
% phi, theta, psi: Euler angles (rad)

% Trimmed Controls:
% throttle       : Engine throttle [0-1]
% elevator       : Elevator deflection (rad)
% aileron        : Aileron deflection (rad)
% rudder         : Rudder deflection (rad)
% nozzle_pitch   : TVC pitch deflection (rad)