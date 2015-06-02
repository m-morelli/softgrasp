% GraspForceOptimizer.OPTIMIZE. Attempt to Solve the Grasp Force Optimization Problem
%   Description, in progress
%
%   Annotations, in progress
%
%   References, in progress
%
%   Matteo Morelli, I.R.C. "E. Piaggio", University of Pisa
%
%   May 2012
%   Modified on October 2013:
%       - code aligned with the new class organization

% Copyright (C) 2012, 2013 Interdepartmental Research Center "E. Piaggio", University of Pisa
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

function [gOutFC] = optimize(gfo, wExt)

    % check no of rhs
    narginchk(2,2);

    % check type, size, etc.
    if ~isa(gfo, 'GraspForceOptimizer'),
        error('TRG:optimize:NonConsistentValue',...
            'Wrong input argument, expected a GraspForceOptimizer object.');
    end
    if numel(gfo) ~= 1,
        error('TRG:optimize:NonConsistentValue',...
            'Wrong input argument: expected a GraspForceOptimizer, not an array of GraspForceOptimizers.');
    end
    if ~isa(wExt, 'numeric'),
        error('TRG:optimize:NonConsistentValue', ...
            'Wrong input argument, expected an external wrench vector (wExt).');
    end
    wExt = real(wExt(:));
    rp = gfo.pReducedProb;
    if numel(wExt) ~= rp.dim,
        error('TRG:optimize:NonConsistentValue', ...
            'Wrong wrench vector, expected %d elements in place of %d.', ...
                rp.dim, numel(wExt));
    end

    % init. settings
    gfo.wExt = wExt;
    hh = 1;
    yx = [];
    vx = [];
    stepSolver = gfo.stepSolver;
    y = gfo.y0;

    % compute the descent direction (Newton's method)
    [v,dv,hv,s] = gfo.llcs_vdvhv(y);
    d = -hv\dv; %d = -dv;

    % compute the step length
    [la] = stepSolver(gfo, y, d);

    % update y
    y_k = y + la*d;

    % save all in the return variables
    yx = [yx, y_k];
    vx = [vx, v];
    sx = s;

    % compute the new value of cost functional
    [v_k,dv_k,hv_k,s] = gfo.llcs_vdvhv(y_k);

    %repeat until not the best
    while (abs(v_k - v) > v*gfo.tol) && (hh < gfo.max_stepn),

        % save the old value
        v = v_k;

        % compute the new descent direction (Newton's method)
        d_k = -hv_k\dv_k; %d_k = -dv_k;

        % compute the new step length
        [la] = stepSolver(gfo, y_k, d_k);

        % update y
        y_k = y_k + la*d_k;

        % save all in the return variables
        yx = [yx, y_k]; %#ok<*AGROW>
        vx = [vx, v];
        sx = char(sx, s);

        % compute the new value of cost functional
        [v_k,dv_k,hv_k,s] = gfo.llcs_vdvhv(y_k);

        % +1 step
        hh = hh + 1;

    end

    if hh == gfo.max_stepn,
        error('TRG:optimize:NonConvergentSolution', ...
                'The algorithm wouldn''t converge after %d steps.', ...
                    hh);
    end

    % prepare the output
    gOutFC = GrOutForceClosure(...
                    'y', yx, ...
                    'v', vx, ...
                    'nSteps', hh, ...
                    'log', sx);
end