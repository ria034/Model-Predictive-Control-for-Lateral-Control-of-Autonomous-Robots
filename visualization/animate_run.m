function animate_run(res, video_file)
%ANIMATE_RUN Animate a run: car, path, obstacles and the MPC's predicted offset.
%   animate_run(res)                    play in a figure
%   animate_run(res, 'results/run.mp4') also write a video
if nargin < 2, video_file = ''; end
L = res.log; p = res.plant; path = res.path;
fig = figure('Name', 'Animation', 'Color', 'w', 'Position', [100 100 1000 650]);
ax = axes(fig); draw_track(ax, path);
trail = plot(ax, nan, nan, 'Color', [0 0.447 0.741], 'LineWidth', 1.5);
predl = plot(ax, nan, nan, '.-', 'Color', [0.93 0.69 0.13], 'MarkerSize', 8);
car = patch(ax, nan, nan, [0 0.447 0.741], 'FaceAlpha', 0.5, 'EdgeColor', [0 0.3 0.6]);
wheel = plot(ax, nan, nan, 'r-', 'LineWidth', 3);
ttl = title(ax, '');
if ~isempty(video_file)
    vw = VideoWriter(video_file, 'MPEG-4'); vw.FrameRate = 20; open(vw);
end
lc = 4.6; wc = 1.9; view_r = 35;
for k = 1:numel(L.t)
    c = cos(L.psi(k)); s = sin(L.psi(k));
    bx = [-0.45 0.55 0.55 -0.45]*lc; by = [-0.5 -0.5 0.5 0.5]*wc;
    set(car, 'XData', L.X(k) + c*bx - s*by, 'YData', L.Y(k) + s*bx + c*by);
    fx = L.X(k) + c*p.a; fy = L.Y(k) + s*p.a; dh = L.psi(k) + L.delta(k);
    set(wheel, 'XData', fx + [-0.4 0.4]*cos(dh), 'YData', fy + [-0.4 0.4]*sin(dh));
    set(trail, 'XData', L.X(1:k), 'YData', L.Y(1:k));
    if all(isfinite(L.pred_ey(:, k)))
        Np = size(L.pred_ey, 1); px = zeros(Np, 1); py = px; ss = L.s(k);
        for q = 1:Np
            j = path_index(path, ss); ss = ss + path.v(j)*res.cfg.sim.Ts;
            j = path_index(path, ss);
            px(q) = path.x(j) - sin(path.psi(j))*L.pred_ey(q, k);
            py(q) = path.y(j) + cos(path.psi(j))*L.pred_ey(q, k);
        end
        set(predl, 'XData', px, 'YData', py);
    end
    xlim(ax, L.X(k) + [-view_r view_r]); ylim(ax, L.Y(k) + [-view_r view_r]*0.65);
    ttl.String = sprintf('%s   t = %.1f s   v = %.1f m/s   e_y = %+.2f m   \\delta = %+.1f deg', ...
        strrep(res.controller, '_', ' '), L.t(k), L.vx(k), L.ey(k), rad2deg(L.delta(k)));
    drawnow;
    if ~isempty(video_file), writeVideo(vw, getframe(fig)); else, pause(0.01); end
end
if ~isempty(video_file), close(vw); fprintf('Saved %s\n', video_file); end
end
