function [F, c, I] = mr_damper_model(I_cmd, vrel, p)
%MR_DAMPER_MODEL  Controllable viscous MR damper (slide 4).
%   c(I) = c_min + (c_max - c_min) * I / I_max      (current saturated to [0, I_max])
%   F    = c(I) * vrel                              (vrel = zs_dot - zu_dot)
%   Because c >= c_min > 0, F always has the sign of vrel, so the damper only
%   dissipates energy (F*vrel >= 0): it is genuinely semi-active.
I = min(max(I_cmd, 0), p.I_max);
c = p.c_min + (p.c_max - p.c_min) * I / p.I_max;
F = c * vrel;
end
