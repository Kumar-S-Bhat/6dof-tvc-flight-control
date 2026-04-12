function controller = generate_dlqr_gain(op)
    %% LQR SAS for Rotational Dynamics
    % Designs LQR on [p, q, r] states using actuator inputs [δe,δa,δr,δn]
    %
    % Input:
    %   op - Operating point from findop
    %
    % Output:
    %   controller.K  : 3x3 gain from [p;q;r] to [δe,δa,δr,δn]
    
    fprintf('=== SAS LQR DESIGN ===\n');
    
    model = 'aircraft_6dof';
    load_system(model);

    %% Verify Linearization I/O Points
    io = getlinio(model);
    if isempty(io)
        error('No linearization points found!\n');
    end

    fprintf('✓ Found %d linearization I/O points\n', length(io));

    %% Linearization
    sys_full = linearize(model, io, op);

    % Extract aircraft states (12) and actuators (5)
    A_full = sys_full.A;
    
    aircraft_idx = 11:22;                     % [u,v,w,x,y,z,p,q,r,phi,theta,psi]
    actuator_pos_idx = [1, 3, 5, 7, 9];      % [aileron, elevator, nozzle, rudder, throttle]
    
    A = A_full(aircraft_idx, aircraft_idx);   % 12x12
    B = A_full(aircraft_idx, actuator_pos_idx); % 12x5
    
    % Reorder B to [elevator, aileron, rudder, throttle, nozzle]
    B = B(:,[2 1 4 5 3]);
    
    % SAS Design: Fast Rotational Dynamics Only 
    % States: p (7), q (8), r (9)
    A_sas = A([7, 8, 9], [7, 8, 9]);        % 3x3
    B_sas = B([7,8,9], [1,2,3,5]);          % 3x4

    Ts = 0.008;   % 125 Hz sample time

    % Discretize the SAS subsystem
    sys_sas_c = ss(A_sas, B_sas, eye(3), zeros(3,4));
    sys_sas_d = c2d(sys_sas_c, Ts);   % ZOH discretization
    
    Ad_sas = sys_sas_d.A;   % 3x3
    Bd_sas = sys_sas_d.B;   % 3x4
    
    % LQR weights
    Q = diag([1, 1, 1]);                % penalize p, q, r
    R = diag([1, 1, 1, 5]);             % nozzle more expensive
    
    %  Design discrete lqr (dlqr)
    [K, ~, P] = dlqr(Ad_sas, Bd_sas, Q, R);

    % Stability check: discrete poles must be INSIDE unit circle
    if all(abs(P) < 1)
        fprintf('✓ Discrete LQR stable, all poles inside unit circle\n');
        fprintf('%f %f %f',P(1),P(2),P(3))
    else
        warning('Closed loop has poles OUTSIDE unit circle!');
    end

    % --- Package Results ---
    controller.K = K;                  % 4x3: [δe,δa,δr,δn] = -K * [p;q;r]
    
    % Store design parameters
    controller.Ts = Ts;       % Sample Time
    controller.Ad = Ad_sas;
    controller.Bd = Bd_sas;
    controller.Q = Q;
    controller.R = R;
    controller.trim_op = op;
    controller.linearization_info.model = model;
    controller.linearization_info.date = datetime('now');

    % Save to workspace and file
    assignin('base', 'controller', controller);
    if ~exist('data', 'dir'), mkdir('data'); end
    save('data/dlqr_gain.mat', 'controller');

    fprintf('\n=== COMPLETE ===\n');
    fprintf('✓ Gain schedule saved to: data/lqr_gain.mat\n');
    fprintf('✓ Loaded to workspace as "controller"\n\n');
end

