function draw_track(ax, path)
%DRAW_TRACK Centreline, road edges and obstacles.
[xl, yl, xr, yr] = road_edges(path);
hold(ax, 'on');
plot(ax, xl, yl, '-', 'Color', [0.55 0.55 0.55], 'HandleVisibility', 'off');
plot(ax, xr, yr, '-', 'Color', [0.55 0.55 0.55], 'HandleVisibility', 'off');
plot(ax, path.x, path.y, 'k--', 'LineWidth', 0.8, 'DisplayName', 'centreline');
th = linspace(0, 2*pi, 60);
for k = 1:numel(path.obstacles)
    o = path.obstacles(k); j = path_index(path, o.s);
    cx = path.x(j) - sin(path.psi(j))*o.d; cy = path.y(j) + cos(path.psi(j))*o.d;
    fill(ax, cx + o.r*cos(th), cy + o.r*sin(th), [0.85 0.33 0.10], 'EdgeColor', 'none', ...
        'FaceAlpha', 0.8, 'HandleVisibility', 'off');
end
axis(ax, 'equal'); grid(ax, 'on');
xlabel(ax, 'X (m)'); ylabel(ax, 'Y (m)');
end
