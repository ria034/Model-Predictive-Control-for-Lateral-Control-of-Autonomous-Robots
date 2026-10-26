function T = run_benchmark(varargin)
%RUN_BENCHMARK Every controller on every scenario under every robustness case.
%   T = run_benchmark()                       full benchmark (takes a while: NMPC dominates)
%   T = run_benchmark('controllers', {'stanley','nmpc'}, 'scenarios', {'hairpin_wet'})
%   T = run_benchmark('seeds', 1:3)           repeat noisy cases with several seeds
%   Writes results/benchmark_runs.csv, results/benchmark_summary.csv,
%   results/benchmark.mat and figures in results/figures/.

root = setup_paths();
ip = inputParser;
ip.addParameter('controllers', {'pure_pursuit','stanley','lqr','lmpc','ampc','nmpc'});
ip.addParameter('scenarios', {'lane_change','s_curves','hairpin_wet','obstacles'});
ip.addParameter('cases', {'nominal','noise','mismatch','delay','delay_nocomp','combined'});
ip.addParameter('seeds', 1);
ip.addParameter('plot', true);
ip.parse(varargin{:});
o = ip.Results;

rows = {};
total = numel(o.scenarios)*numel(o.controllers)*numel(o.cases)*numel(o.seeds);
done = 0; t0 = tic;
for si = 1:numel(o.scenarios)
    for ci = 1:numel(o.controllers)
        for ki = 1:numel(o.cases)
            for seed = o.seeds
                cfg = apply_case(default_config(), o.cases{ki});
                cfg.robust.seed = seed;
                res = run_scenario(cfg, o.scenarios{si}, o.controllers{ci});
                m = res.metrics;
                rows(end+1, :) = {o.scenarios{si}, o.controllers{ci}, o.cases{ki}, seed, m.status, ...
                    m.rms_ey, m.max_ey, m.rms_epsi_deg, m.max_delta_deg, m.steer_rate_rms, ...
                    m.max_ay, m.min_clear, m.collision, m.t_end, m.comp_ms_mean, m.comp_ms_max}; %#ok<AGROW>
                done = done + 1;
                fprintf('[%3d/%3d] %-12s %-13s %-13s seed %d  %-10s rms e_y %.3f m  (%.0f s elapsed)\n', ...
                    done, total, o.scenarios{si}, o.controllers{ci}, o.cases{ki}, seed, m.status, m.rms_ey, toc(t0));
            end
        end
    end
end
T = cell2table(rows, 'VariableNames', {'scenario','controller','case_name','seed','status', ...
    'rms_ey','max_ey','rms_epsi_deg','max_delta_deg','steer_rate_rms','max_ay','min_clear', ...
    'collision','t_end','comp_ms_mean','comp_ms_max'});

outdir = fullfile(root, 'results');
writetable(T, fullfile(outdir, 'benchmark_runs.csv'));
S = groupsummary(T, {'scenario','controller','case_name'}, 'mean', ...
    {'rms_ey','max_ey','max_delta_deg','steer_rate_rms','min_clear','comp_ms_mean'});
writetable(S, fullfile(outdir, 'benchmark_summary.csv'));
save(fullfile(outdir, 'benchmark.mat'), 'T', 'S');
fprintf('Saved results to %s\n', outdir);
if o.plot, plot_benchmark(T, fullfile(outdir, 'figures')); end
end
