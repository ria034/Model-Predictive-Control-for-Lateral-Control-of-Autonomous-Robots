function root = setup_paths()
%SETUP_PATHS Add the project folders to the MATLAB path. Run once per session
%   from the repository root:   setup_paths
root = fileparts(mfilename('fullpath'));
addpath(root);
folders = {'config', 'models', 'controllers', 'utils', 'main', 'visualization'};
for k = 1:numel(folders)
    addpath(fullfile(root, folders{k}));
end
if ~exist(fullfile(root, 'results'), 'dir'), mkdir(fullfile(root, 'results')); end
end
