function trim_result = trim_solver(V_target, h_target, gamma_target)
%% 6DOF Trim Solver with Robust Optimization
    % Finds equilibrium for full 6DOF aircraft
    % Solves for: [throttle, delta_e, delta_a, delta_r, delta_p, alpha, beta, phi]
    
    % Initial guess
    throttle_guess = 0.05;
    delta_e_guess = 0.0;
    delta_a_guess = 0.0;
    delta_r_guess = 0.0;
    delta_p_guess = 0.0;
    alpha_guess = deg2rad(1);
    beta_guess = 0.0;
    phi_guess = 0.0;
    
    x0 = [throttle_guess, delta_e_guess, delta_a_guess, delta_r_guess, ...
        delta_p_guess, alpha_guess, beta_guess, phi_guess];
    
    % Bounds
    lb = [0.0,  -deg2rad(25), -deg2rad(21.5), -deg2rad(30), -deg2rad(20), -deg2rad(10), -deg2rad(5), -deg2rad(5)];
    ub = [1.0,   deg2rad(25),  deg2rad(21.5),  deg2rad(30),  deg2rad(20),  deg2rad(20),  deg2rad(5),  deg2rad(5)];
    
    % Optimization options
    options = optimoptions('lsqnonlin', ...
        'Display', 'iter', ...
        'MaxIterations', 1000, ...
        'MaxFunctionEvaluations', 5000, ...
        'FunctionTolerance', 1e-10, ...
        'StepTolerance', 1e-10, ...
        'FiniteDifferenceStepSize', 1e-8 ...
        );
    
    % Solve
    fprintf('Starting trim solver for V=%.1f m/s, h=%.1f m, γ=%.2f deg...\n', ...
        V_target, h_target, rad2deg(gamma_target));
    
    tic;
    [x_opt, resnorm, residual, exitflag] = lsqnonlin(...
        @(x) trim_residuals(x, V_target, h_target, gamma_target), ...
        x0, lb, ub, options);
    solve_time = toc;
    
    % Extract results
    trim_result.throttle = x_opt(1);
    trim_result.delta_e = x_opt(2);
    trim_result.delta_a = x_opt(3);
    trim_result.delta_r = x_opt(4);
    trim_result.delta_p = x_opt(5);
    trim_result.alpha = x_opt(6);
    trim_result.beta = x_opt(7);
    trim_result.phi = x_opt(8);
    trim_result.V = V_target;
    trim_result.h = h_target;
    trim_result.gamma = gamma_target;
    trim_result.residuals = residual;
    trim_result.max_residual = max(abs(residual));
    trim_result.solve_time = solve_time;
    trim_result.success = (exitflag > 0) && (trim_result.max_residual < 1e-3);
    
    % Calculate full initial state
    trim_result.theta = gamma_target + trim_result.alpha;
    trim_result.psi = 0;
    trim_result.u = V_target * cos(trim_result.alpha) * cos(trim_result.beta);
    trim_result.v = V_target * sin(trim_result.beta);
    trim_result.w = V_target * sin(trim_result.alpha) * cos(trim_result.beta);
    trim_result.p = 0;
    trim_result.q = 0;
    trim_result.r = 0;
    
    % Display results
    fprintf('\n========== TRIM SOLUTION ==========\n');
    fprintf('  Success:          %s\n', string(trim_result.success));
    fprintf('  Max Residual:     %.2e\n', trim_result.max_residual);
    fprintf('  Solve Time:       %.2f s\n', trim_result.solve_time);
    
    fprintf('\nControls:\n');
    fprintf('  Throttle:  %.4f  (%.1f%%)\n', trim_result.throttle, ...
        trim_result.throttle*100);
    fprintf('  Elevator:  %+.2f deg\n', rad2deg(trim_result.delta_e));
    fprintf('  Aileron:   %+.2f deg\n', rad2deg(trim_result.delta_a));
    fprintf('  Rudder:    %+.2f deg\n', rad2deg(trim_result.delta_r));
    fprintf('  Nozzle:    %+.2f deg\n', rad2deg(trim_result.delta_p));
    
    fprintf('\nState:\n');
    fprintf('  Alpha:     %+.2f deg\n', rad2deg(trim_result.alpha));
    fprintf('  Beta:      %+.2f deg\n', rad2deg(trim_result.beta));
    fprintf('  Theta:     %+.2f deg\n', rad2deg(trim_result.theta));
    fprintf('  Phi:       %+.2f deg\n', rad2deg(trim_result.phi));
    
    fprintf('\nResiduals:\n');
    fprintf('  [Fx Fy Fz]: [%+.2e %+.2e %+.2e]\n', residual(1:3));
    fprintf('  [Mx My Mz]: [%+.2e %+.2e %+.2e]\n', residual(4:6));
    fprintf('========================================\n\n');
    
    if ~trim_result.success
        warning('Trim quality poor. Check initial guess and bounds.');
    end

end

%% Residual function
function residuals = trim_residuals(x, V_target, h_target, gamma_target)
    % Unpack variables
    throttle = x(1);
    delta_e = x(2);
    delta_a = x(3);
    delta_r = x(4);
    delta_p = x(5);
    alpha = x(6);
    beta = x(7);
    phi = x(8);
    
    % Load parameters
    aircraft = evalin('base', 'aircraft');
    aero = evalin('base', 'aero');
    engine = evalin('base', 'engine');
    atmos = evalin('base', 'atmos');
    
    % State
    theta = gamma_target + alpha;
    psi = 0;
    u = V_target * cos(alpha) * cos(beta);
    v = V_target * sin(beta);
    w = V_target * sin(alpha) * cos(beta);
    p = 0; q = 0; r = 0;
    
    state = [u; v; w; p; q; r];
    controls = [delta_e; delta_a; delta_r];
    
    % Atmosphere - Returns density, temperature, pressure, speed of sound
    [T, a, P, rho] = atmosisa(h_target);
    
    % Aerodynamics
    [F_aero, M_aero] = aerodynamics_trim(state, controls, rho, aero, aircraft);
    
    % Thrust
    T = engine.thrust_max * throttle * (rho / atmos.rho_sl);
    Fx_thrust = T * cos(delta_p);
    Fz_thrust = T * sin(delta_p);
    My_thrust = Fz_thrust * engine.nozzle_arm;
    
    F_thrust = [Fx_thrust; 0; Fz_thrust];
    M_thrust = [0; My_thrust; 0];
    
    % Total forces
    F_total = F_aero + F_thrust;
    
    % Gravity
    g = atmos.g;
    m = aircraft.mass;
    C_bn = euler_to_dcm(phi, theta, psi);
    F_grav_ned = [0; 0; m*g];
    F_grav_body = C_bn * F_grav_ned;
    
    % Net forces and moments
    F_net = F_total + F_grav_body;
    M_net = M_aero + M_thrust;
    
    % Normalized residuals
    residuals = [
        F_net(1) / (m*g);
        F_net(2) / (m*g);
        F_net(3) / (m*g);
        M_net(1) / (m*g*aircraft.b);
        M_net(2) / (m*g*aircraft.c);
        M_net(3) / (m*g*aircraft.b);
    ];

end

%% Helper functions
function C_bn = euler_to_dcm(phi, theta, psi)
    cphi = cos(phi); sphi = sin(phi);
    cth = cos(theta); sth = sin(theta);
    cpsi = cos(psi); spsi = sin(psi);
    
    C_bn = [cth*cpsi, cth*spsi, -sth;
            sphi*sth*cpsi - cphi*spsi, sphi*sth*spsi + cphi*cpsi, sphi*cth;
            cphi*sth*cpsi + sphi*spsi, cphi*sth*spsi - sphi*cpsi, cphi*cth];
end

function [F_aero, M_aero] = aerodynamics_trim(state, controls, rho, aero, ...
    aircraft)
    % Unpack
    u = state(1); v = state(2); w = state(3);
    p = state(4); q = state(5); r = state(6);
    delta_e = controls(1);
    delta_a = controls(2);
    delta_r = controls(3);
    
    % Flight parameters
    V = sqrt(u^2 + v^2 + w^2);
    if V < 1e-6, V = 1e-6; end
    alpha = atan2(w, u);
    beta = asin(v / V);
    
    % Dynamic pressure
    qbar = 0.5 * rho * V^2;
    
    % Clamp alpha
    alpha_clamped = max(aero.alpha_min, min(alpha, aero.alpha_max));

    
    % Interpolate coefficients
    CL = ppval(aero.CL_table, alpha_clamped);
    CD = ppval(aero.CD_table, alpha_clamped);
    Cm = ppval(aero.Cm_table, alpha_clamped);
    Cm_de = ppval(aero.Cm_de_table, alpha_clamped);
    
    %{
    CL = aero.CL_interp(alpha_clamped);
    CD = aero.CD_interp(alpha_clamped);
    Cm = aero.Cm_interp(alpha_clamped);
    Cm_de = aero.Cm_de_interp(alpha_clamped);
    %}

    % Wind-frame forces
    D = qbar * aircraft.S * CD;
    L = qbar * aircraft.S * CL;
    Y = qbar * aircraft.S * (aero.Cy_beta * beta + aero.Cy_dr * delta_r);
    
    % Transform to body frame
    ca = cos(alpha); sa = sin(alpha);
    cb = cos(beta);  sb = sin(beta);
    
    Fx_aero = -D * ca * cb - Y * sb + L * sa * cb;
    Fy_aero = -D * ca * sb + Y * cb + L * sa * sb;
    Fz_aero =  D * sa - L * ca;
    
    F_aero = [Fx_aero; Fy_aero; Fz_aero];
    
    % Moments
    Cm_total = Cm + aero.Cm_q * (q * aircraft.c / (2 * V)) + Cm_de * delta_e;
    My_aero = qbar * aircraft.S * aircraft.c * Cm_total;
    
    Cl_total = aero.Cl_beta * beta + aero.Cl_da * delta_a + ...
               aero.Cl_p * (p * aircraft.b / (2 * V)) + ...
               aero.Cl_r * (r * aircraft.b / (2 * V));
    Mx_aero = qbar * aircraft.S * aircraft.b * Cl_total;
    
    Cn_total = aero.Cn_beta * beta + aero.Cn_dr * delta_r + ...
               aero.Cn_p * (p * aircraft.b / (2 * V)) + ...
               aero.Cn_r * (r * aircraft.b / (2 * V));
    Mz_aero = qbar * aircraft.S * aircraft.b * Cn_total;
    
    M_aero = [Mx_aero; My_aero; Mz_aero];
end

%{
function rho = atmosphere_isa(altitude, atmos)
    if altitude < 11000
        T = 288.15 - 0.0065 * altitude;
        rho = atmos.rho_sl * (T / 288.15)^4.256;
    else
        rho = atmos.rho_sl * (216.65 / 288.15)^4.256;
    end
end
%}