function fig = plot_run(res)
%PLOT_RUN Detailed plots for one simulation result.
L = res.log; p = res.plant;
fig = figure('Name', sprintf('%s on %s', res.controller, res.scenario), 'Color', 'w', 'Position', [80 80 1200 760]);
tl = tiledlayout(fig, 3, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, sprintf('%s on %s (%s)', strrep(res.controller, '_', ' '), strrep(res.scenario, '_', ' '), res.status));

ax = nexttile(tl, [2 2]);
draw_track(ax, res.path);
plot(ax, L.X, L.Y, 'Color', [0 0.447 0.741], 'LineWidth', 1.6, 'DisplayName', 'vehicle');
legend(ax, 'Location', 'best'); title(ax, 'Path');

ax = nexttile(tl);
lo = zeros(size(L.s)); hi = lo; rf = lo;
for k = 1:numel(L.s), [lo(k), hi(k), rf(k)] = corridor_bounds(res.path, L.s(k), res.nominal, true); end
plot(ax, L.s, L.ey, 'LineWidth', 1.3); hold(ax, 'on');
plot(ax, L.s, lo, 'r--', L.s, hi, 'r--'); plot(ax, L.s, rf, 'k:');
grid(ax, 'on'); xlabel(ax, 's (m)'); ylabel(ax, 'e_y (m)'); title(ax, 'Lateral offset and corridor');

ax = nexttile(tl);
plot(ax, L.t, rad2deg(L.u), 'Color', [0.85 0.33 0.10]); hold(ax, 'on');
plot(ax, L.t, rad2deg(L.delta), 'Color', [0 0.447 0.741], 'LineWidth', 1.3);
yline(ax, rad2deg(p.delta_max), 'r--'); yline(ax, -rad2deg(p.delta_max), 'r--');
grid(ax, 'on'); xlabel(ax, 't (s)'); ylabel(ax, '\delta (deg)');
legend(ax, {'command', 'actual'}, 'Location', 'best'); title(ax, 'Steering');

ax = nexttile(tl);
plot(ax, L.t, L.vref, 'k--', L.t, L.vx, 'LineWidth', 1.2);
grid(ax, 'on'); xlabel(ax, 't (s)'); ylabel(ax, 'v_x (m/s)'); legend(ax, {'profile', 'actual'}, 'Location', 'best');
title(ax, 'Speed');

ax = nexttile(tl);
Fzf = p.m*p.g*p.b/p.L; Fzr = p.m*p.g*p.a/p.L;
asf = rad2deg(atan(3*p.mu*Fzf/p.Cf)); asr = rad2deg(atan(3*p.mu*Fzr/p.Cr));
plot(ax, L.t, rad2deg(L.af), L.t, rad2deg(L.ar)); hold(ax, 'on');
yline(ax, asf, '--', 'front saturates'); yline(ax, -asf, '--');
yline(ax, asr, ':', 'rear saturates'); yline(ax, -asr, ':');
grid(ax, 'on'); xlabel(ax, 't (s)'); ylabel(ax, '\alpha (deg)'); legend(ax, {'front', 'rear'}, 'Location', 'best');
title(ax, 'Tyre slip angles');

ax = nexttile(tl);
plot(ax, L.t, L.ay, 'LineWidth', 1.2); hold(ax, 'on');
yline(ax, p.mu*p.g, 'r--'); yline(ax, -p.mu*p.g, 'r--');
grid(ax, 'on'); xlabel(ax, 't (s)'); ylabel(ax, 'a_y (m/s^2)'); title(ax, 'Lateral acceleration vs. \mu g');
end
