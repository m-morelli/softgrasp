% LLC_GFO. Low-Level Computational Routine for Grasp Force Optimization
%   Description, in progress
%
%   Annotations, in progress
%
%   References, in progress
%
%   Matteo Morelli, I.R.C. "E. Piaggio", University of Pisa
%
%   May 2012

% Copyright (C) 2012 Interdepartmental Research Center "E. Piaggio", University of Pisa
%
% This file is part of THE Robotic Grasping Toolbox for use with MATLAB(R).
%
% THE Robotic Grasping Toolbox for use with MATLAB(R) is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
%
% THE Robotic Grasping Toolbox for use with MATLAB(R) is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
%
% You should have received a copy of the GNU General Public License
% along with THE Robotic Grasping Toolbox for use with MATLAB(R).  If not, see <http://www.gnu.org/licenses/>.

function [gOutFC] = llc_gfo(optGFO)

% init. settings
tol = 1e-10;
max_stepn = 1000;
hh = 1;
yx = [];
vx = [];
flag = optGFO.stepSolver;
y = optGFO.y0;

% compute the descent direction (Newton's method)
[v,dv,hv,s] = llcs_vdvhv(optGFO, y);
d = -hv\dv; %d = -dv;

% compute the step length
[la] = llcs_vdvhvss(optGFO, y, d, flag);

% update y
y_k = y + la*d;

% save all in the return variables
yx = [yx, y_k];
vx = [vx, v];
sx = s;

% compute the new value of cost functional
[v_k,dv_k,hv_k,s] = llcs_vdvhv(optGFO, y_k);

%repeat until not the best
while (abs(v_k - v) > v*tol) && (hh < max_stepn),

    % save the old value
    v = v_k;

    % compute the new descent direction (Newton's method)
    d_k = -hv_k\dv_k; %d_k = -dv_k;

    % compute the new step length
    [la] = llcs_vdvhvss(optGFO, y_k, d_k, flag);

    % update y
    y_k = y_k + la*d_k;

    % save all in the return variables
    yx = [yx, y_k]; %#ok<*AGROW>
    vx = [vx, v];
    sx = char(sx, s);

    % compute the new value of cost functional
    [v_k,dv_k,hv_k,s] = llcs_vdvhv(optGFO, y_k);

    % +1 step
    hh = hh + 1;

end

if hh == max_stepn,
    error('GRASP:classProps:NonConvergentSolution', ...
            'the algorithm wouldn''t converge after %d steps', ...
                hh);
end

% prepare the output
gOutFC = GrOutForceClosure(...
                'y', yx, ...
                'v', vx, ...
                'nSteps', hh, ...
                'log', sx);