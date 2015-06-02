disp('- Robotics Toolbox for Matlab (release 9)')
tbpath = fileparts(which('Link'));
addpath( fullfile(tbpath, 'demos') );
addpath( fullfile(tbpath, 'examples') );
javaaddpath( fullfile(tbpath, 'DH.jar') );
