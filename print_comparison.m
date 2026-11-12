function print_comparison(titleStr, mP, mS, mD)
%PRINT_COMPARISON  Command-window table: passive vs Skyhook vs PID.
rows = {'peak_zs_mm','Peak body displacement [mm]';
        'peak_acc','Peak body acceleration [m/s^2]';
        'rms_acc','RMS body acceleration [m/s^2]';
        'rms_travel_mm','RMS suspension travel [mm]';
        'rms_tyre_mm','RMS tyre deflection [mm]';
        'peak_tyre_mm','Peak tyre deflection [mm]';
        'settle_s','Settling time [s]'};
iS = compare_metrics(mP, mS);  iD = compare_metrics(mP, mD);
fprintf('\n=== %s ===\n', titleStr);
fprintf('%-34s %10s %10s %8s %10s %8s\n', 'Metric', 'Passive', 'Skyhook', 'impr.%', 'PID', 'impr.%');
for i = 1:size(rows,1)
    k = rows{i,1};
    if isnan(mP.(k)), continue; end
    fprintf('%-34s %10.3f %10.3f %+7.1f%% %10.3f %+7.1f%%\n', rows{i,2}, mP.(k), mS.(k), iS.(k), mD.(k), iD.(k));
end
end
