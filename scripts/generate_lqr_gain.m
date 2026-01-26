function controller = generate_lqr_gain(op)
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
    
    % --- SAS Design: Fast Rotational Dynamics Only ---
    % States: p (7), q (8), r (9)
    A_sas = A([7, 8, 9], [7, 8, 9]);        % 3x3
    B_sas = B([7,8,9], [1,2,3,5]);          % 3x4

    % LQR weights
    Q = diag([1, 1, 1]);                % penalize p, q, r
    R = diag([1, 1, 1, 5]);             % nozzle more expensive
    
    % Desing LQR gian
    [K,~,P] = lqr(A_sas,B_sas,Q,R);

    % Stability check
    if all(real(P)<0)
        fprintf('LQR designed successfully');
        fprintf('%f %f %f',P(1),P(2),P(3))
    else
        warning('Closed Loop has unstable poles');
    end

    % --- Package Results ---
    controller.K = K;                  % 4x3: [δe,δa,δr,δn] = -K * [p;q;r]
    
    % Store design parameters
    controller.Q = Q;
    controller.R = R;
    controller.trim_op = op;
    controller.linearization_info.model = model;
    controller.linearization_info.date = datetime('now');

    % Save to workspace and file
    assignin('base', 'controller', controller);
    if ~exist('data', 'dir'), mkdir('data'); end
    save('data/lqr_gain.mat', 'controller');

    fprintf('\n=== COMPLETE ===\n');
    fprintf('✓ Gain schedule saved to: data/lqr_gain.mat\n');
    fprintf('✓ Loaded to workspace as "controller"\n\n');
end

