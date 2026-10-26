function m = compute_metrics(res)
%COMPUTE_METRICS Tracking, effort, safety and timing metrics for one run.
L = res.log; Ts = res.cfg.sim.Ts;
m.status = res.status;
m.finished = strcmp(res.status, 'finished');
m.rms_ey = sqrt(mean(L.ey.^2));                 % from the centreline (includes planned offsets)
m.max_ey = max(abs(L.ey));
m.rms_epsi_deg = rad2deg(sqrt(mean(L.epsi.^2)));
m.max_delta_deg = rad2deg(max(abs(L.delta)));
m.steer_rate_rms = sqrt(mean(diff(L.delta).^2))/Ts;   % rad/s, smoothness
m.max_ay = max(abs(L.ay));
m.min_clear = min(L.clear);                     % m, Inf when there are no obstacles
m.collision = m.min_clear < 0;
m.t_end = L.t(end);
m.comp_ms_mean = 1000*mean(L.comp);
m.comp_ms_max = 1000*max(L.comp);
end
