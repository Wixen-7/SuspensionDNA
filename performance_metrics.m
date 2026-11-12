function m = performance_metrics(res, p, roadType)
%PERFORMANCE_METRICS  RMS / peak / settling metrics for one simulation.
m.peak_zs_mm     = max(abs(res.zs))     * 1e3;
m.peak_acc       = max(abs(res.as));
m.rms_acc        = sqrt(mean(res.as.^2));
m.rms_travel_mm  = sqrt(mean(res.travel.^2)) * 1e3;
m.peak_travel_mm = max(abs(res.travel)) * 1e3;
m.rms_tyre_mm    = sqrt(mean(res.tyre.^2)) * 1e3;
m.peak_tyre_mm   = max(abs(res.tyre))   * 1e3;
m.rms_force_N    = sqrt(mean(res.F.^2));
m.mean_current_A = mean(res.I);
m.settle_s       = NaN;
if nargin >= 3 && strcmpi(roadType, 'bump')
    t_end = p.bump_t0 + p.bump_T;                       % time after the bump
    idx = find(abs(res.zs) > p.settle_band & res.t > t_end, 1, 'last');
    if isempty(idx), m.settle_s = 0; else, m.settle_s = res.t(idx) - t_end; end
end
end
