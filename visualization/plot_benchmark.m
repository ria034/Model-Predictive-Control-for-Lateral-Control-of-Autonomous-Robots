function plot_benchmark(T, outdir)
%PLOT_BENCHMARK Heatmaps and bar charts of a benchmark table (from run_benchmark).
%   plot_benchmark(T)            show figures
%   plot_benchmark(T, outdir)    also save PNGs into outdir
if nargin < 2, outdir = ''; end
if ~isempty(outdir) && ~exist(outdir, 'dir'), mkdir(outdir); end
scens = unique(T.scenario, 'stable'); ctrls = unique(T.controller, 'stable'); cases = unique(T.case_name, 'stable');
nice = @(c) strrep(c, '_', ' ');

% 1) RMS lateral error, one heatmap per scenario (path-tracking scenarios)
track = setdiff(scens, {'obstacles'}, 'stable');
fig = figure('Name', 'RMS lateral error', 'Color', 'w', 'Position', [40 40 1300 380*ceil(numel(track)/2)]);
tl = tiledlayout(fig, ceil(numel(track)/2), 2, 'TileSpacing', 'compact');
title(tl, 'RMS lateral error (m); red text = did not finish');
for si = 1:numel(track)
    [M, F] = grid_of(T, track{si}, ctrls, cases, 'rms_ey');
    ax = nexttile(tl); heat(ax, M, F, ctrls, cases); title(ax, nice(track{si}));
end
save_fig(fig, outdir, 'rms_ey_heatmaps.png');

% 2) obstacle clearance
if any(strcmp(scens, 'obstacles'))
    [M, F] = grid_of(T, 'obstacles', ctrls, cases, 'min_clear');
    fig = figure('Name', 'Obstacle clearance', 'Color', 'w', 'Position', [60 60 900 420]);
    b = bar(categorical(nice(ctrls), nice(ctrls)), M); grid on;
    ylabel('minimum clearance (m), < 0 = collision'); yline(0, 'r-', 'LineWidth', 1.5);
    legend(b, nice(cases), 'Location', 'bestoutside'); title('Obstacle scenario');
    save_fig(fig, outdir, 'obstacle_clearance.png');
end

% 3) computation time
fig = figure('Name', 'Computation time', 'Color', 'w', 'Position', [80 80 700 380]);
ct = zeros(numel(ctrls), 1);
for ci = 1:numel(ctrls), ct(ci) = mean(T.comp_ms_mean(strcmp(T.controller, ctrls{ci}))); end
bar(categorical(nice(ctrls), nice(ctrls)), ct); grid on;
ylabel('mean time per control step (ms)'); yline(1000*0.05, 'r--', 'sample time 50 ms');
title('Computation time (this PC, MATLAB)');
save_fig(fig, outdir, 'computation_time.png');
end

function [M, F] = grid_of(T, scen, ctrls, cases, field)
M = nan(numel(ctrls), numel(cases)); F = false(size(M));
for ci = 1:numel(ctrls)
    for ki = 1:numel(cases)
        sel = strcmp(T.scenario, scen) & strcmp(T.controller, ctrls{ci}) & strcmp(T.case_name, cases{ki});
        if any(sel)
            M(ci, ki) = mean(T.(field)(sel));
            F(ci, ki) = any(~strcmp(T.status(sel), 'finished'));
        end
    end
end
end

function heat(ax, M, F, ctrls, cases)
imagesc(ax, log10(max(M, 1e-3))); colormap(ax, parula); cb = colorbar(ax);
cb.Label.String = 'log_{10}(m)';
set(ax, 'XTick', 1:numel(cases), 'XTickLabel', strrep(cases, '_', ' '), ...
        'YTick', 1:numel(ctrls), 'YTickLabel', strrep(ctrls, '_', ' '), 'XTickLabelRotation', 25);
for i = 1:size(M, 1)
    for j = 1:size(M, 2)
        if F(i, j), col = [0.85 0 0]; else, col = [1 1 1]; end
        text(ax, j, i, sprintf('%.2f', M(i, j)), 'HorizontalAlignment', 'center', 'Color', col, 'FontWeight', 'bold');
    end
end
end

function save_fig(fig, outdir, name)
if ~isempty(outdir), exportgraphics(fig, fullfile(outdir, name), 'Resolution', 150); end
end
