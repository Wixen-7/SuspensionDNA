%% MAIN  Runs the complete passive vs MR semi-active comparison (no GUI).
%   Section 1: validation checks
%   Section 2: bump test        (reproduces slide 6)
%   Section 3: random road test (reproduces slide 7, mean of 5 seeds)
%   Section 4: figures + results.mat
%   Run dashboard.m afterwards for the animated version.
clear; clc; close all;
p = vehicle_parameters();

%% 1. Sanity checks (Phase 1)
assert(abs(damping_to_current(p.c_min,p)) < 1e-12 && abs(damping_to_current(p.c_max,p) - p.I_max) < 1e-12, ...
       'c<->I map wrong');
fprintf('Natural frequencies: body %.2f Hz, wheel-hop %.2f Hz\n', ...
        sqrt(p.ks/p.ms)/(2*pi), sqrt((p.ks+p.kt)/p.mu)/(2*pi));
fprintf('Passive damping ratio (body): %.2f\n', p.c_passive/(2*sqrt(p.ks*p.ms)));

%% 2. Bump test
road = road_profile('bump', p, 5);
rP = simulate_suspension('passive', road, p);
rS = simulate_suspension('skyhook', road, p);
rD = simulate_suspension('pid',     road, p);
assert(rS.energy_ok && rD.energy_ok, 'MR damper injected energy!');
assert(all(rS.I >= 0 & rS.I <= p.I_max) && all(rD.I >= 0 & rD.I <= p.I_max), 'Current out of range');
mP = performance_metrics(rP, p, 'bump');
mS = performance_metrics(rS, p, 'bump');
mD = performance_metrics(rD, p, 'bump');
print_comparison('BUMP (5 cm, 0.2 s half-sine at t = 1 s)', mP, mS, mD);

figure('Name','Bump response','Position',[80 80 1100 800]);
subplot(2,2,1); plot(rP.t,rP.zs*1e3,rS.t,rS.zs*1e3,rD.t,rD.zs*1e3,'LineWidth',1.3); grid on
ylabel('Body displacement [mm]'); legend('Passive','Skyhook','PID'); title('Bump response'); xlim([0 4])
subplot(2,2,2); plot(rP.t,rP.as,rS.t,rS.as,rD.t,rD.as,'LineWidth',1.3); grid on
ylabel('Body acceleration [m/s^2]'); xlim([0 4])
subplot(2,2,3); plot(rP.t,rP.tyre*1e3,rS.t,rS.tyre*1e3,rD.t,rD.tyre*1e3,'LineWidth',1.3); grid on
ylabel('Tyre deflection [mm]'); xlabel('t [s]'); xlim([0 4])
subplot(2,2,4); yyaxis left; plot(rS.t,rS.I,'LineWidth',1.3); ylabel('Skyhook current [A]');
yyaxis right; plot(rS.t,rS.F,'LineWidth',1.0); ylabel('MR force [N]'); grid on; xlabel('t [s]'); xlim([0 4])
saveas(gcf,'bump_response.png');

%% 3. Random road, 30 s, 5 seeds, identical road for all three suspensions
nSeeds = 5;  T = 30;
acc = struct('P',[],'S',[],'D',[]);
for s = 1:nSeeds
    rr = road_profile('random', p, T, s);
    acc.P = [acc.P; cell2mat(struct2cell(performance_metrics(simulate_suspension('passive',rr,p), p, 'random')))'];
    acc.S = [acc.S; cell2mat(struct2cell(performance_metrics(simulate_suspension('skyhook',rr,p), p, 'random')))'];
    acc.D = [acc.D; cell2mat(struct2cell(performance_metrics(simulate_suspension('pid',    rr,p), p, 'random')))'];
    fprintf('  random seed %d/%d done\n', s, nSeeds);
end
names = fieldnames(mP);
avg = @(M) cell2struct(num2cell(mean(M,1)'), names, 1);
rP_m = avg(acc.P); rS_m = avg(acc.S); rD_m = avg(acc.D);
print_comparison(sprintf('RANDOM ROAD (ISO 8608, %d m/s, %d s, mean of %d seeds)', p.V, T, nSeeds), rP_m, rS_m, rD_m);

iS = compare_metrics(rP_m, rS_m);  iD = compare_metrics(rP_m, rD_m);
figure('Name','Random-road improvement','Position',[120 120 700 450]);
bar([iS.rms_acc iS.rms_travel_mm iS.rms_tyre_mm; iD.rms_acc iD.rms_travel_mm iD.rms_tyre_mm]'); grid on
set(gca,'XTickLabel',{'RMS body accel.','RMS susp. travel','RMS tyre deflection'});
ylabel('Improvement vs passive [%]'); legend('Skyhook','PID','Location','southwest');
title('Random road: comfort vs road-holding trade-off'); saveas(gcf,'random_improvement.png');

%% 4. Save
save('results.mat','p','rP','rS','rD','mP','mS','mD','rP_m','rS_m','rD_m');
disp('Saved results.mat, bump_response.png, random_improvement.png');
