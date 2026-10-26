%MAIN_DEMO Run the controllers on one scenario and compare them.
%   Change `scenario`, `controllers` or the cfg lines below and run again.

addpath(fullfile(fileparts(mfilename('fullpath')), '..'));
root = setup_paths();

scenario = 'hairpin_wet';      % 'lane_change' | 's_curves' | 'hairpin_wet' | 'obstacles'
controllers = {'pure_pursuit', 'stanley', 'lqr', 'lmpc', 'ampc', 'nmpc'};

cfg = default_config();
% cfg = apply_case(cfg, 'combined');   % noise + model mismatch + 100 ms delay
% cfg.vehicle = vehicle_params('repo'); % the vehicle from the original Simulink model

results = cell(size(controllers));
for k = 1:numel(controllers)
    fprintf('Running %s on %s ... ', controllers{k}, scenario);
    results{k} = run_scenario(cfg, scenario, controllers{k});
    m = results{k}.metrics;
    fprintf('%s, RMS e_y %.3f m, max e_y %.2f m, %.1f ms per step\n', m.status, m.rms_ey, m.max_ey, m.comp_ms_mean);
end

plot_comparison(results);
plot_run(results{end});
% animate_run(results{end});                                         % watch it
% animate_run(results{end}, fullfile(root, 'results', 'nmpc.mp4'));   % save a video
