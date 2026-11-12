function [I, c_des] = pid_controller(zs, vs, as_prev, vrel, p)
%PID_CONTROLLER  PID on body motion, mapped onto a semi-active damper.
%   Reference = body at rest. Desired damper force
%       F_des = Kp*zs_dot + Ki*zs + Kd*zs_ddot
%   (zs is the integral of zs_dot, so Ki*zs is the integral term; using the
%   state directly avoids integrator wind-up.)
%   Semi-active constraint: F = c*vrel with c in [c_min, c_max], so F_des can
%   only be followed when it has the same sign as vrel.
F_des = p.pid.Kp * vs + p.pid.Ki * zs + p.pid.Kd * as_prev;
if abs(vrel) > 1e-6 && F_des * vrel > 0
    c_des = min(max(F_des / vrel, p.c_min), p.c_max);
else
    c_des = p.c_min;
end
I = damping_to_current(c_des, p);
end
