function load_aero_data()
%% Load Aerodynamic Coefficients
% Creates lookup tables for CL, CD, Cm, Cn, Cl, Cy
    
    % Angle of attack range (radians for Simulink lookup tables)
    alpha_deg = [-10 -5 0 5 10 15 18 20 25 30 35 40 45 50 55 60 65 70 75 80 85 90]';
    alpha_rad = deg2rad(alpha_deg);
    aero.alpha_table = alpha_rad;
    %% Longitudinal Coefficients
    
    % Lift coefficient
    aero.CL_data = [
        -0.80; -0.35; 0.25; 0.80; 1.25; 1.50; 1.45; 1.30;
        1.00; 0.80; 0.65; 0.50; 0.42; 0.35; 0.30; 0.26;
        0.23; 0.22; 0.20; 0.18; 0.10; 0.00
    ];
    
    % Drag coefficient
    aero.CD_data = [
        0.080; 0.045; 0.020; 0.025; 0.040; 0.080; 0.130; 0.190;
        0.380; 0.650; 0.950; 1.300; 1.600; 1.800; 1.950; 2.050;
        2.150; 2.250; 2.310; 2.350; 2.380; 2.400
    ];
    
    % Pitch moment coefficient (static)
    aero.Cm_data = [
        -0.050; -0.030; 0.005; 0.010; 0.015; 0.018; 0.015; 0.008;
        -0.005; -0.015; -0.022; -0.028; -0.032; -0.035; -0.037; -0.038;
        -0.039; -0.040; -0.041; -0.042; -0.042; -0.040
    ];
    
    % Elevator effectiveness (Cm_de) vs alpha
    aero.Cm_de_data = [
        -1.20; -1.20; -1.20; -1.20; -1.15; -1.00; -0.80; -0.60;
        -0.40; -0.30; -0.20; -0.15; -0.10; -0.08; -0.06; -0.05;
        -0.04; -0.03; -0.02; -0.01; -0.005; 0.00
    ];
    
    % Pitch damping (Cm_q) - dimensionless
    aero.Cm_q = -15.0;

    %% Lateral-Directional Coefficients (simplified - constants)
    
    % Side force
    aero.Cy_beta = -0.98;   % per radian (side force due to sideslip)
    aero.Cy_dr = 0.175;     % per radian (side force due to rudder)
    
    % Roll moment
    aero.Cl_beta = -0.13;   % per radian (dihedral effect)
    aero.Cl_da = 0.229;     % per radian (roll due to aileron)
    aero.Cl_p = -0.484;     % roll damping
    aero.Cl_r = 0.0798;     % roll due to yaw rate
    
    % Yaw moment
    aero.Cn_beta = 0.071;   % per radian (directional stability)
    aero.Cn_dr = -0.115;    % per radian (yaw due to rudder)
    aero.Cn_p = -0.0278;    % adverse yaw
    aero.Cn_r = -0.15;      % yaw damping

    % spline interpolation
    aero.CL_table = spline(alpha_rad, aero.CL_data);
    aero.CD_table = spline(alpha_rad, aero.CD_data);
    aero.Cm_table = spline(alpha_rad, aero.Cm_data);
    aero.Cm_de_table = spline(alpha_rad, aero.Cm_de_data);
%{
    aero.CL_interp = griddedInterpolant(aero.alpha_table, aero.CL_data, 'spline');
    aero.CD_interp = griddedInterpolant(aero.alpha_table, aero.CD_data, 'spline');
    aero.Cm_interp = griddedInterpolant(aero.alpha_table, aero.Cm_data, 'spline');
    aero.Cm_de_interp = griddedInterpolant(aero.alpha_table, aero.Cm_de_data, 'spline');
%}

    % Keep bounds for clamping
    aero.alpha_min = alpha_rad(1);
    aero.alpha_max = alpha_rad(end);

    
    fprintf('✓ Aerodynamic data loaded\n');
    fprintf('  Alpha range: %.0f to %.0f deg\n', alpha_deg(1), alpha_deg(end));
    fprintf('  CL max: %.2f @ %.0f deg\n', max(aero.CL_data), alpha_deg(aero.CL_data == max(aero.CL_data)));
    fprintf('  Lateral-directional derivatives loaded\n');

    assignin('base','aero',aero)
end