function I = damping_to_current(c, p)
%DAMPING_TO_CURRENT  Inverse of the linear c(I) map (slide 5, "damping to current").
c = min(max(c, p.c_min), p.c_max);
I = p.I_max * (c - p.c_min) / (p.c_max - p.c_min);
end
