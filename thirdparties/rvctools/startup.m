disp('Robotics, Vision & Control: (c) Peter Corke 1992-2011 http://www.petercorke.com')
tb = false;
rvcpath = fileparts( mfilename('fullpath') );
if exist('robot')
    addpath( fullfile(rvcpath, 'robot') )
    tb = true;
    startup_rtb
end
if exist('vision')
    %addpath( fullfile(rvcpath, 'vision') )
    %tb = true;
    %startup_mvtb
end

if tb
    addpath( fullfile(pwd, 'common') )
    addpath( fullfile(pwd, 'simulink') )
end
