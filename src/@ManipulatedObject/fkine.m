%ManipulatedObject.FKINE
%   Description, in progress
%
%   Annotations, in progress
%
%   References, in progress
%
%   Matteo Morelli, I.R.C. "E. Piaggio", University of Pisa
%
%   December 2012

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

function T = fkine(~, u, reducedProb_)

    reducedProb = GrOpt2dProblem();
    if nargin > 2,
        if ~isa(reducedProb_, 'GrOpt2dProblem'),
            error('GRASP:classProps:NonConsistentValue', ...
                'wrong value specified for ''reducedProb'', expected a GrOpt2dProblem object');
        end
        reducedProb = reducedProb_;
    end
    ri = srv_compute_3d_orientation_of_manip_object(u, reducedProb);
    ti = srv_compute_3d_position_of_manip_object(u, reducedProb);
    T = rt2tr(ri, ti);