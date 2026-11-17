% STARTUP  Add the project folders to the MATLAB path.
% Run this once per session (MATLAB runs it automatically if you open
% MATLAB in this folder).
root = fileparts(mfilename('fullpath'));
addpath(fullfile(root, 'src', 'data'), fullfile(root, 'src', 'sim'), ...
        fullfile(root, 'src', 'util'), fullfile(root, 'demos'), ...
        fullfile(root, 'experiments'), fullfile(root, 'tests'));
clear root
