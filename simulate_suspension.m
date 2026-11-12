function res = simulate_suspension(ctrl, road, p)
%SIMULATE_SUSPENSION  Closed-loop quarter-car simulation (RK4, fixed step).
%   ctrl : 'passive' | 'skyhook' | 'pid'
%   State x = [zs; zu; zs_dot; zu_dot]
%       ms*zs'' = -ks*(zs-zu) - F
%       mu*zu'' =  ks*(zs-zu) + F - kt*(zu-zr)         F = c*(zs_dot-zu_dot)
%   The controller is sampled once per step (zero-order hold).
dt = p.dt;
N  = round(road.T / dt);
x  = zeros(4, 1);
I_act = 0;  as_prev = 0;

zs = zeros(N+1,1); zu = zs; vs = zs; vu = zs; as = zs;
F = zs; c = zs; I = zs;

for k = 1:N+1
    ia = 2*k - 1;  zr0 = road.zr(ia);              % road at start of step
    vrel = x(3) - x(4);

    % ---- controller -> command current ------------------------------------
    switch lower(ctrl)
        case 'passive'
            I_cmd = NaN;
        case 'skyhook'
            I_cmd = skyhook_controller(x(3), vrel, p);
        case 'pid'
            I_cmd = pid_controller(x(1), x(3), as_prev, vrel, p);
        otherwise
            error('simulate_suspension: unknown controller "%s"', ctrl);
    end

    % ---- actuator ---------------------------------------------------------
    if strcmpi(ctrl, 'passive')
        c_k = p.c_passive;  I_k = 0;
    else
        if p.tau_mr > 0
            I_act = I_act + dt / p.tau_mr * (I_cmd - I_act);
        else
            I_act = I_cmd;
        end
        [~, c_k, I_k] = mr_damper_model(I_act, vrel, p);
    end

    % ---- log --------------------------------------------------------------
    d1 = deriv(x, zr0, c_k, p);
    zs(k) = x(1); zu(k) = x(2); vs(k) = x(3); vu(k) = x(4);
    as(k) = d1(3);  as_prev = d1(3);
    c(k) = c_k;  I(k) = I_k;  F(k) = c_k * vrel;
    if k == N+1, break; end

    % ---- RK4 (road read at start, mid and end of step) ---------------------
    zr_mid = road.zr(ia + 1);  zr_end = road.zr(ia + 2);
    d2 = deriv(x + dt/2*d1, zr_mid, c_k, p);
    d3 = deriv(x + dt/2*d2, zr_mid, c_k, p);
    d4 = deriv(x + dt*d3,   zr_end, c_k, p);
    x  = x + dt/6 * (d1 + 2*d2 + 2*d3 + d4);
end

res.ctrl = lower(ctrl);
res.t    = (0:N)' * dt;
res.zs = zs; res.zu = zu; res.vs = vs; res.vu = vu; res.as = as;
res.zr = road.zr(1:2:2*N+1);
res.travel = zs - zu;              % suspension travel
res.tyre   = zu - res.zr;          % tyre deflection
res.F = F; res.c = c; res.I = I;
res.vrel = vs - vu;
res.power = F .* res.vrel;         % damper power (>= 0 means pure dissipation)
res.energy_ok = all(res.power >= -1e-9);
end

function d = deriv(x, zr, c, p)
F = c * (x(3) - x(4));
d = [ x(3);
      x(4);
     (-p.ks*(x(1)-x(2)) - F) / p.ms;
     ( p.ks*(x(1)-x(2)) + F - p.kt*(x(2)-zr)) / p.mu ];
end
