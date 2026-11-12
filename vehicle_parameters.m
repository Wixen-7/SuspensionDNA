function p = vehicle_parameters()
%VEHICLE_PARAMETERS  All model, controller and test parameters in one place.
%   Values are taken from slide 4/5/6/7 of MR_Damper_Project.pptx.

% --- Quarter-car ---------------------------------------------------------
p.ms = 300;        % sprung mass (body)            [kg]
p.mu = 40;         % unsprung mass (wheel)         [kg]
p.ks = 18000;      % suspension spring stiffness   [N/m]
p.kt = 180000;     % tyre stiffness                [N/m]

% --- MR damper -----------------------------------------------------------
p.c_min     = 800;     % damping at 0 A            [N.s/m]
p.c_max     = 5000;    % damping at I_max          [N.s/m]
p.I_max     = 2;       % max coil current          [A]
p.c_passive = 1500;    % conventional damper       [N.s/m]
p.tau_mr    = 0;       % MR response lag [s]. 0 = ideal (matches the slides);
                       % try 0.01 for a realistic ~10 ms response.

% --- Controllers ---------------------------------------------------------
p.c_sky = 4000;        % Skyhook gain              [N.s/m]
p.pid.Kp = 6000;       % PID on body motion (desired force = Kp*vs + Ki*zs + Kd*as)
p.pid.Ki = 20000;
p.pid.Kd = 200;

% --- Road / test set-up --------------------------------------------------
p.V        = 20;        % vehicle speed                 [m/s]
p.Gd0      = 128e-6;    % road roughness Gd(n0=0.1/m)   [m^3]  (ISO 8608 B/C limit;
                        % this value reproduces the absolute RMS values on slide 7)
p.bump_h   = 0.05;      % bump height                   [m]
p.bump_T   = 0.2;       % bump duration (half-sine)     [s]
p.bump_t0  = 1.0;       % bump start time               [s]
p.settle_band = 2e-3;   % settling band for |zs|        [m]

% --- Numerics ------------------------------------------------------------
p.dt = 1e-3;            % fixed RK4 step                [s]
end
