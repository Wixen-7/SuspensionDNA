function road = road_profile(type, p, T, seed)
%ROAD_PROFILE  Bump or random road, sampled on a HALF-step grid (dt/2) so the
%   RK4 solver can read start / mid / end of each step exactly.
%   type : 'bump' | 'random'
%   T    : duration [s];   seed : RNG seed for 'random' (same road is reused
%          for every suspension, so the comparison is fair).
if nargin < 4, seed = 1; end
t   = (0:p.dt/2:T)';
zr  = zeros(size(t));
zrd = zeros(size(t));

switch lower(type)
    case 'bump'
        w = pi / p.bump_T;
        m = t >= p.bump_t0 & t <= p.bump_t0 + p.bump_T;
        zr(m)  = p.bump_h * sin(w * (t(m) - p.bump_t0));
        zrd(m) = p.bump_h * w * cos(w * (t(m) - p.bump_t0));

    case 'random'
        % ISO 8608 road: Gd(n) = Gd0*(n/n0)^-2, synthesised as a sum of cosines
        % with random phases. Velocity is analytic (no numerical differentiation).
        s   = RandStream('mt19937ar', 'Seed', seed);
        dn  = 0.005;                       % spatial-frequency spacing [cycles/m]
        n   = (0.01:dn:5)';                % 0.01 ... 5 cycles/m (up to 100 Hz at 20 m/s)
        A   = sqrt(2 * p.Gd0 * (n / 0.1).^(-2) * dn);
        phi = 2 * pi * rand(s, numel(n), 1);
        w   = 2 * pi * n * p.V;            % rad/s
        chunk = 100;
        for i = 1:chunk:numel(n)
            j   = i:min(i + chunk - 1, numel(n));
            arg = t * w(j)' + repmat(phi(j)', numel(t), 1);
            zr  = zr  + cos(arg) * A(j);
            zrd = zrd - sin(arg) * (A(j) .* w(j));
        end
    otherwise
        error('road_profile: unknown road type "%s"', type);
end

road.type = lower(type);
road.t    = t;
road.zr   = zr;
road.zrd  = zrd;
road.T    = T;
road.seed = seed;
end
