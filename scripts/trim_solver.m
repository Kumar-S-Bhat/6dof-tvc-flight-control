function [trim_result,op,opreport] = trim_solver(V_target, h_target)    
    %% Specify the model name
    model = 'aircraft_6dof';
    
    %% Create the operating point specification object.
    opspec = operspec(model);

    %% Set the constraints on the states in the model.
    % Set actuator state constraints (States 1-5)
    for i = 1:5
        opspec.States(i).x = [0; 0];
        opspec.States(i).Known = [false; true];  % Position free, velocity = 0
    end
    
    % State (6) - aircraft_6dof/state-space model/6DOF (Euler Angles)
    opspec.States(6).x = [V_target;0;0;0;0;-h_target;0;0;0;0;0;0];
    opspec.States(6).Known = [false;true;false;false;true;true;true;true;true;true;false;true];
    opspec.States(6).SteadyState = [true;true;true;false;true;true;true;true;true;true;true;true];

    %% Set the constraints on the inputs in the model.
    
    % Input (1) - aircraft_6dof/elevator
    opspec.Inputs(1).u = 0;
    opspec.Inputs(1).Min = -deg2rad(25);
    opspec.Inputs(1).Max = deg2rad(25);
    
    % Input (2) - aircraft_6dof/aileron
    opspec.Inputs(2).u = 0;
    opspec.Inputs(2).Min = -deg2rad(21.5);
    opspec.Inputs(2).Max = deg2rad(21.5);
    
    % Input (3) - aircraft_6dof/rudder
    opspec.Inputs(3).u = 0;
    opspec.Inputs(3).Min = -deg2rad(30);
    opspec.Inputs(3).Max = deg2rad(30);
    
    % Input (4) - aircraft_6dof/throttle
    opspec.Inputs(4).u = 0;
    opspec.Inputs(4).Min = 0;
    opspec.Inputs(4).Max = 1;
    
    % Input (5) - aircraft_6dof/delta_p
    opspec.Inputs(5).u = 0;
    opspec.Inputs(5).Min = -deg2rad(20);
    opspec.Inputs(5).Max = deg2rad(20);
    
    %% Set the constraints on the outputs in the model.
    
    % Output (1) - aircraft_6dof/V
    opspec.Outputs(1).y = V_target;
    opspec.Outputs(1).Known = true;
    
    % Output (2) - aircraft_6dof/h
    opspec.Outputs(2).y = -h_target;
    opspec.Outputs(2).Known = true;
    
    opspec.Outputs(3).Known = false;
    opspec.Outputs(4).Known = false;
    
    
    %% Create the options
    opt = findopOptions('DisplayReport','off');
    opt.OptimizationOptions.MaxFunEvals = 4000;
    opt.OptimizationOptions.MaxIter = 1000;
    opt.OptimizationOptions.TolFun = 1.000000e-08;
    
    %% Perform the operating point search.
    [op,opreport] = findop(model,opspec,opt);

    if ~strcmpi(opreport.TerminationString, 'Operating point specifications were successfully met.')
        error('Trim failed: %s\n', opreport.TerminationString);
    end

    %% Extract 6DOF states
    state_fields = {'u', 'v', 'w', 'x', 'y', 'z', 'p', 'q', 'r', 'phi', 'theta', 'psi'};
    for i = 1:length(state_fields)
        trim_result.(state_fields{i}) = op.States(6).x(i);
    end

    %% Extract control inputs
    control_fields = {'elevator', 'aileron', 'rudder', 'throttle', 'nozzle_pitch'};
    for i = 1:length(control_fields)
        trim_result.(control_fields{i}) = op.Inputs(i).u;
    end

    trim_result.airspeed = opreport.Outputs(1).y;
end