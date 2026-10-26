function cfg = apply_case(cfg, name)
%APPLY_CASE Robustness test cases used by run_benchmark.
switch name
    case 'nominal'
    case 'noise'
        cfg.robust.noise = true;
    case 'mismatch'                 % heavier car, worn tyres, 15 % less grip than assumed
        cfg.robust.mismatch = true;
        cfg.robust.mu_scale = 0.85;
    case 'delay'                    % 100 ms steering delay, compensated by model-based controllers
        cfg.robust.delay_steps = 2;
    case 'delay_nocomp'             % same delay, no compensation
        cfg.robust.delay_steps = 2;
        cfg.robust.compensate_delay = false;
    case 'combined'
        cfg.robust.noise = true;
        cfg.robust.mismatch = true;
        cfg.robust.mu_scale = 0.85;
        cfg.robust.delay_steps = 2;
    otherwise
        error('apply_case:name', 'Unknown case "%s".', name);
end
end
