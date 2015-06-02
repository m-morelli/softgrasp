% GraspAnalysisSolver.PLOT. Plot a Grasp
%   Description, in progress
%
%   Annotations, in progress
%
%   References, in progress
%
%   Matteo Morelli, I.R.C. "E. Piaggio", University of Pisa
%
%   August 2013

% Copyright (C) 2013 Interdepartmental Research Center "E. Piaggio", University of Pisa
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

function plot(grsp, fc_, q_, u_)

    % rhs args ?
    narginchk(1,4);

    % basic checks
    if ~isa(grsp, 'GraspAnalysisSolver'),
        error('TRG:plot:NonConsistentValue', ...
            'Wrong input argument, expected a GraspAnalysisSolver object.');
    end

    % shortcuts
    rp = grsp.pReducedProb;

    % defaults
    fc = [];
    q = (grsp.q)';
    u = (grsp.u)';
    rp_u_dim = rp.dim;
    if grsp.isProblem2d && ...  % is this really needed?
            numel(u) > rp_u_dim,
        u = (rp.reduceObjectComponentsTo2d(grsp.u))';
    end

    % additional inputs
    % no checks (will be performed by the superclass plot() method)
    if nargin > 1,
        fc = fc_;
        if nargin > 2,
            q = q_;            
            if nargin > 3,
                u = u_;
            end
        end
    end

    % do plot!
    plot@GraspSolver(grsp, q, u, fc);

end
