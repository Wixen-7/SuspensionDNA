function out = animation(action, varargin)
%ANIMATION  Draws one animated quarter-car (body, wheel, spring, MR damper, tyre spring, road).
%   h = animation('create', ax, gain)       build graphics in axes ax
%   animation('update', h, s)                move them; s has fields
%        zs, zu [m]      body / wheel displacement
%        roadY  [m]      road height window around the tyre (1 x nWin)
%        Iratio [0..1]   damper current / I_max (colours the damper)
%        txt    string   caption inside the axes
%   gain = visual exaggeration of displacements (purely cosmetic).
switch lower(action)
    case 'create', out = create_car(varargin{:});
    case 'update', update_car(varargin{:}); out = [];
end
end

% ---------------------------------------------------------------------------
function h = create_car(ax, gain)
cla(ax); hold(ax,'on');
ax.XLim = [-1.4 1.4]; ax.YLim = [-0.25 1.8];
ax.XTick = []; ax.YTick = []; ax.DataAspectRatio = [1 1 1]; box(ax,'on');
h.gain = gain;  h.xs = linspace(-1.3, 1.3, 81);          % road window x positions
h.yw0 = 0.28;  h.rw = 0.28;  h.yb0 = 1.00;  h.bh = 0.34;  % rest geometry
h.road_fill = patch(ax, nan, nan, [0.86 0.83 0.76], 'EdgeColor','none');
h.road      = plot(ax, nan, nan, '-', 'Color',[0.25 0.25 0.25], 'LineWidth',2.5);
h.tyre_spr  = plot(ax, nan, nan, '-', 'Color',[0.9 0.5 0.1], 'LineWidth',1.6);
th = linspace(0, 2*pi, 50);  h.th = th;
h.wheel     = plot(ax, nan, nan, '-', 'Color',[0.1 0.1 0.1], 'LineWidth',3);
h.axle      = patch(ax, nan, nan, [0.45 0.45 0.45], 'EdgeColor','k');
h.spring    = plot(ax, nan, nan, '-', 'Color',[0.1 0.55 0.2], 'LineWidth',1.8);
h.cyl       = patch(ax, nan, nan, [0.2 0.4 0.9], 'EdgeColor','k', 'LineWidth',1.2);
h.rod       = plot(ax, nan, nan, '-', 'Color',[0.3 0.3 0.3], 'LineWidth',3);
h.body      = patch(ax, nan, nan, [0.55 0.7 0.95], 'EdgeColor','k', 'LineWidth',1.5);
h.txt       = text(ax, 0, 1.68, '', 'HorizontalAlignment','center', 'FontSize',10, 'FontWeight','bold');
text(ax, -1.35, -0.18, sprintf('displacements x%g', gain), 'FontSize',8, 'Color',[0.4 0.4 0.4]);
end

% ---------------------------------------------------------------------------
function update_car(h, s)
g  = h.gain;
yw = h.yw0 + g*s.zu;                    % wheel hub height
yb = h.yb0 + g*s.zs;                    % body bottom height
yr = g * s.roadY;                       % road window heights
yc = yr(round(numel(yr)/2));            % road height under the tyre

set(h.road,      'XData', h.xs, 'YData', yr);
set(h.road_fill, 'XData', [h.xs fliplr(h.xs)], 'YData', [yr -0.25*ones(size(yr))]);
set(h.tyre_spr,  zigzag(0, yc, yw, 7, 0.10));
set(h.wheel,     'XData', h.rw*cos(h.th), 'YData', yw + h.rw*sin(h.th));
set(h.axle,      'XData', [-0.3 0.3 0.3 -0.3], 'YData', yw + [0 0 0.08 0.08]);
ytop = yw + 0.08;
set(h.spring,    zigzag(-0.25, ytop, yb, 8, 0.09));
cylTop = ytop + 0.30;
set(h.cyl,       'XData', 0.25 + 0.06*[-1 1 1 -1], 'YData', [ytop ytop cylTop cylTop], ...
                 'FaceColor', (1-s.Iratio)*[0.25 0.45 0.9] + s.Iratio*[0.9 0.2 0.2]);
set(h.rod,       'XData', [0.25 0.25], 'YData', [cylTop-0.12 yb]);
set(h.body,      'XData', [-0.55 0.55 0.55 -0.55], 'YData', yb + h.bh*[0 0 1 1]);
set(h.txt,       'String', s.txt);
end

% ---------------------------------------------------------------------------
function s = zigzag(x0, y0, y1, nCoil, w)
% Vertical spring from (x0,y0) to (x0,y1) with nCoil zig-zags.
n  = 2*nCoil;
yy = linspace(y0, y1, n + 2);
xx = x0 + [0, w*(-1).^(1:n), 0];
s  = struct('XData', xx, 'YData', yy);
end
