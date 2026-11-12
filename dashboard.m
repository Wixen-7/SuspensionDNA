function dashboard()
%DASHBOARD  Interactive passive vs MR semi-active quarter-car simulator.
%   Pick road + controller, press Run, then Play. Both cars get exactly the same
%   road. Needs MATLAB R2018b+ (uifigure). Run main.m first if you only want numbers.
p      = vehicle_parameters();
gain   = 4;                 % visual exaggeration of displacements
nWin   = 81;                % samples in the animated road window
resP = []; resM = []; road = []; mP = []; mM = [];
idx = 1; playing = false; startIdx = 1; tStart = tic;
cursors = gobjects(1,4);

fig = uifigure('Name','MR damper semi-active suspension simulator','Position',[40 40 1320 800]);

% ---- control panel -------------------------------------------------------
pc = uipanel(fig,'Title','Simulation control','Position',[10 700 1300 90]);
uilabel(pc,'Text','Road','Position',[10 38 40 22]);
ddRoad = uidropdown(pc,'Items',{'Bump','Random road'},'Position',[50 38 110 22],'ValueChangedFcn',@(~,~) runSim());
uilabel(pc,'Text','Controller','Position',[175 38 65 22]);
ddCtrl = uidropdown(pc,'Items',{'Skyhook','PID'},'Position',[240 38 90 22],'ValueChangedFcn',@(~,~) runSim());
uilabel(pc,'Text','Seed','Position',[345 38 35 22]);
spSeed = uispinner(pc,'Value',1,'Limits',[1 999],'Step',1,'RoundFractionalValues','on','Position',[380 38 60 22],'ValueChangedFcn',@(~,~) runSim());
btnRun   = uibutton(pc,'Text','Run','Position',[460 36 70 26],'ButtonPushedFcn',@(~,~) runSim());
btnPlay  = uibutton(pc,'state','Text','Play','Position',[540 36 70 26],'ValueChangedFcn',@(s,~) onPlay(s.Value));
btnReset = uibutton(pc,'Text','Reset','Position',[620 36 70 26],'ButtonPushedFcn',@(~,~) onReset());
uilabel(pc,'Text','Speed','Position',[710 38 40 22]);
sldSpeed = uislider(pc,'Limits',[0.1 2],'Value',1,'Position',[755 48 150 3],'MajorTicks',[0.1 0.5 1 2],'ValueChangedFcn',@(~,~) onSpeed());
lblTime  = uilabel(pc,'Text','t = 0.00 s','Position',[940 38 100 22],'FontWeight','bold');
lblInfo  = uilabel(pc,'Text','','Position',[1040 38 250 22],'FontColor',[0.4 0.4 0.4]);
uilabel(pc,'Text','Road window / bump scrolls toward the wheel; body & wheel motion exaggerated for visibility.', ...
        'Position',[10 6 800 20],'FontColor',[0.4 0.4 0.4],'FontSize',10);

% ---- animation axes ------------------------------------------------------
axP = uiaxes(fig,'Position',[10 340 330 350]); title(axP,'Passive  (c = 1500 N.s/m)');
axM = uiaxes(fig,'Position',[350 340 330 350]); title(axM,'MR semi-active');
hP = animation('create', axP, gain);
hM = animation('create', axM, gain);

% ---- live MR readout -----------------------------------------------------
pl = uipanel(fig,'Title','Live MR damper','Position',[10 255 670 75]);
lblI = uilabel(pl,'Text','Current  0.00 A','Position',[15 12 190 24],'FontSize',15,'FontWeight','bold');
lblC = uilabel(pl,'Text','c = 800 N.s/m','Position',[230 12 210 24],'FontSize',15,'FontWeight','bold');
lblF = uilabel(pl,'Text','F = 0 N','Position',[460 12 190 24],'FontSize',15,'FontWeight','bold');

% ---- metrics table -------------------------------------------------------
tbl = uitable(fig,'Position',[690 255 620 435],'ColumnName',{'Metric','Passive','MR','Improvement [%]'}, ...
              'ColumnWidth',{230,100,100,150},'RowName',{});

% ---- response graphs -----------------------------------------------------
ax1 = uiaxes(fig,'Position',[10 10 315 235]);  title(ax1,'Body displacement [mm]');
ax2 = uiaxes(fig,'Position',[335 10 315 235]); title(ax2,'Body acceleration [m/s^2]');
ax3 = uiaxes(fig,'Position',[660 10 315 235]); title(ax3,'Tyre deflection [mm]');
ax4 = uiaxes(fig,'Position',[985 10 325 235]); title(ax4,'MR current [A]');
axs = [ax1 ax2 ax3 ax4];  for a = axs, grid(a,'on'); hold(a,'on'); xlabel(a,'t [s]'); end

runSim();
fig.CloseRequestFcn = @(~,~) onClose();

% =========================================================================
    function runSim()
        playing = false; btnPlay.Value = false; btnPlay.Text = 'Play';
        lblInfo.Text = 'Simulating...'; drawnow;
        isBump = strcmp(ddRoad.Value,'Bump');
        if isBump, road = road_profile('bump',p,5); else, road = road_profile('random',p,15,spSeed.Value); end
        ctrl = lower(ddCtrl.Value);
        resP = simulate_suspension('passive',road,p);
        resM = simulate_suspension(ctrl,road,p);
        mP = performance_metrics(resP,p,road.type);
        mM = performance_metrics(resM,p,road.type);
        fillTable(isBump);
        plotGraphs();
        idx = 1; updateFrame(idx);
        lblInfo.Text = 'Ready - press Play';
        spSeed.Enable = ~isBump;
    end

    function fillTable(isBump)
        imp = compare_metrics(mP,mM);
        rows = {'peak_zs_mm','Peak body displacement [mm]'; 'peak_acc','Peak body accel. [m/s^2]'; ...
                'rms_acc','RMS body accel. [m/s^2]'; 'rms_travel_mm','RMS suspension travel [mm]'; ...
                'rms_tyre_mm','RMS tyre deflection [mm]'; 'peak_tyre_mm','Peak tyre deflection [mm]'; ...
                'rms_force_N','RMS damper force [N]'};
        if isBump, rows(end+1,:) = {'settle_s','Settling time [s]'}; end
        data = cell(size(rows,1),4);
        for i = 1:size(rows,1)
            k = rows{i,1};
            data(i,:) = {rows{i,2}, round(mP.(k),3), round(mM.(k),3), round(imp.(k),1)};
        end
        tbl.Data = data;
    end

    function plotGraphs()
        d = 1:10:numel(resP.t);  t = resP.t(d);
        for a = axs, cla(a); end
        plot(ax1,t,resP.zs(d)*1e3,'Color',[0.3 0.3 0.3],'LineWidth',1.2); plot(ax1,t,resM.zs(d)*1e3,'r','LineWidth',1.2);
        plot(ax2,t,resP.as(d),'Color',[0.3 0.3 0.3],'LineWidth',1.0);     plot(ax2,t,resM.as(d),'r','LineWidth',1.0);
        plot(ax3,t,resP.tyre(d)*1e3,'Color',[0.3 0.3 0.3],'LineWidth',1.0); plot(ax3,t,resM.tyre(d)*1e3,'r','LineWidth',1.0);
        plot(ax4,t,resM.I(d),'r','LineWidth',1.2); ylim(ax4,[-0.05 p.I_max+0.05]);
        legend(ax1,{'Passive','MR'},'Location','northeast');
        for i = 1:4
            xlim(axs(i),[0 t(end)]);
            cursors(i) = plot(axs(i),[0 0],axs(i).YLim,'b--','LineWidth',1);
        end
    end

    function updateFrame(k)
        s.roadY = roadWindow(k);
        s.zs = resP.zs(k); s.zu = resP.zu(k); s.Iratio = 0;
        s.txt = sprintf('z_s = %+.1f mm', resP.zs(k)*1e3);  animation('update',hP,s);
        s.zs = resM.zs(k); s.zu = resM.zu(k); s.Iratio = resM.I(k)/p.I_max;
        s.txt = sprintf('z_s = %+.1f mm', resM.zs(k)*1e3);  animation('update',hM,s);
        t = resP.t(k);
        for i = 1:4, cursors(i).XData = [t t]; cursors(i).YData = axs(i).YLim; end
        lblTime.Text = sprintf('t = %.2f s', t);
        lblI.Text = sprintf('Current  %.2f A', resM.I(k));
        lblC.Text = sprintf('c = %.0f N.s/m', resM.c(k));
        lblF.Text = sprintf('F = %+.0f N', resM.F(k));
    end

    function y = roadWindow(k)
        % road height from -0.5 s (behind) to +0.5 s (ahead) around the tyre
        ih = 2*(k-1) + 1 + round(linspace(-0.5,0.5,nWin)/(p.dt/2));
        ih = min(max(ih,1),numel(road.zr));
        y = road.zr(ih)';
    end

    function onPlay(on)
        if on
            if idx >= numel(resP.t), idx = 1; end
            playing = true; btnPlay.Text = 'Pause';
            startIdx = idx; tStart = tic;
            while playing && isvalid(fig) && idx < numel(resP.t)
                idx = min(numel(resP.t), startIdx + round(sldSpeed.Value*toc(tStart)/p.dt));
                updateFrame(idx); drawnow limitrate;
            end
            if isvalid(fig), playing = false; btnPlay.Value = false; btnPlay.Text = 'Play'; end
        else
            playing = false; btnPlay.Text = 'Play';
        end
    end

    function onSpeed(), startIdx = idx; tStart = tic; end
    function onReset(), playing = false; btnPlay.Value = false; btnPlay.Text = 'Play'; idx = 1; updateFrame(1); end
    function onClose(), playing = false; delete(fig); end
end
