function i = path_index(path, s)
%PATH_INDEX Index of the path sample closest to arc length s (clamped).
i = round(s/path.ds) + 1;
i = min(max(i, 1), numel(path.s));
end
