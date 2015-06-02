%GraspSolver.COMPLETEHANDJACOBIANSPATIAL. Complete Hand Jacobian Matrix in Inertial Frame
%   Description, in progress
%
%   Annotations, in progress
%
%   References, in progress
%
%   Matteo Morelli, I.R.C. "E. Piaggio", University of Pisa
%
%   July 2013

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

function Jtilde = completeHandJacobianSpatial(grsp, q)

    % checking rhs args
    %
    % right number of rhs args?
    narginchk(2,2);
    % check if grsp is a Grasp
    if ~isa(grsp, 'GraspSolver'),
        error('TRG:completeHandJacobianSpatial:NonConsistentValue', ...
            'Wrong input argument, expected a GraspSolver object.');
    end
    if ~isa(q, 'numeric'),
        error('TRG:completeHandJacobianSpatial:NonConsistentValue', ...
            'Wrong input argument, expected a configuration vector (q).');
    end
    q = real(q(:));
    % reduced Prob?
    reducedProb = grsp.pReducedProb;

    % spatial Jacobian
    Jtilde = grsp.handWithGlove.poiDifferentialMapBase(q);

    % eventually reduce to 2D
    if reducedProb.isSet,
        Jtilde = reducedProb.reduceDiffMotionsMapBaseTo2d(Jtilde);
    end