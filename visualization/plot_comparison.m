function fig = plot_comparison(results)
%PLOT_COMPARISON Overlay several controllers on the same scenario.
colors = lines(numel(results));
fig = figure('Name', 'Controller comparison', 'Color', 'w', 'Position', [60 60 1200 720]);
tl = tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, sprintf('Scenario: %s', strrep(results{1}.scenario, '_', ' ')));
ax1 = nexttile(tl, [1 2]); draw_track(ax1, results{1}.path);
ax2 = nexttile(tl); hold(ax2, 'on'); grid(ax2, 'on');
ax3 = nexttile(tl); hold(ax3, 'on'); grid(ax3, 'on');
for k = 1:numel(results)
    r = results{k}; L = r.log;
    name = sprintf('%s (%.2f m RMS)', strrep(r.controller, '_', ' '), r.metrics.rms_ey);
    plot(ax1, L.X, L.Y, 'Color', colors(k,:), 'LineWidth', 1.4, 'DisplayName', name);
    plot(ax2, L.s, L.ey, 'Color', colors(k,:), 'LineWidth', 1.2);
    plot(ax3, L.t, rad2deg(L.delta), 'Color', colors(k,:), 'LineWidth', 1.2);
end
legend(ax1, 'Location', 'bestoutside');
xlabel(ax2, 's (m)'); ylabel(ax2, 'e_y (m)'); title(ax2, 'Lateral offset from centreline');
xlabel(ax3, 't (s)'); ylabel(ax3, '\delta (deg)'); title(ax3, 'Steering angle');
end
