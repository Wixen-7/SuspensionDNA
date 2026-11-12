function [I, c_des] = skyhook_controller(vs, vrel, p)
%SKYHOOK_CONTROLLER  Continuous saturated Skyhook law (slide 5).
%   Ideal skyhook force = c_sky*vs. The real damper can only push the body the
%   right way when vs and vrel have the same sign, otherwise go to c_min.
if vs * vrel > 1e-9
    c_des = min(max(p.c_sky * vs / vrel, p.c_min), p.c_max);
else
    c_des = p.c_min;
end
I = damping_to_current(c_des, p);
end
