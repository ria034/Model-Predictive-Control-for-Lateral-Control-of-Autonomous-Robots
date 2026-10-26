function mem = push_pending(mem, u, cfg)
%PUSH_PENDING Remember the last delay_steps commands (still "in flight" in the actuator delay).
d = cfg.robust.delay_steps;
if d == 0, return; end
if ~isfield(mem, 'pending'), mem.pending = []; end
mem.pending(end+1) = u;
if numel(mem.pending) > d
    mem.pending = mem.pending(end-d+1:end);
end
end
