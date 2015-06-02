%GraspSolver.COMPLETEHANDJACOBIAN. Complete Hand Jacobian Matrix in Object Contact frames
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

function Jtilde = completeHandJacobian(grsp, q, u)

    % checking rhs args
    %
    % right number of rhs args?
    narginchk(3,3);
    % check if grsp is a Grasp
    if ~isa(grsp, 'GraspSolver'),
        error('TRG:completeHandJacobian:NonConsistentValue', ...
            'Wrong input argument, expected a GraspSolver object.');
    end
    if ~isa(q, 'numeric'),
        error('TRG:completeHandJacobian:NonConsistentValue', ...
            'Wrong input argument, expected a configuration vector (q).');
    end
    % check q
    q = real(q(:));
    if ~isa(u, 'numeric'),
        error('TRG:completeHandJacobian:NonConsistentValue', 'Wrong input argument, expected an object configuration vector (u).');
    end
    u = real(u(:));
    % check (eventually adapt) conf
    reducedProb = grsp.pReducedProb;
    if numel(u) ~= reducedProb.dim,
        error('TRG:completeHandJacobian:NonConsistentValue', ...
            'Wrong object config, expected %d elements in place of %d.', ...
                reducedProb.dim, numel(u));
    end
    if reducedProb.isSet,
        u = reducedProb.restoreConfTo3d(u);
    end

    % spatial Jacobian
    Jtilde_s = grsp.handWithGlove.poiDifferentialMapBase(q);

    % Ad_g_ba
    Ad_g_ba = llcs_iad2(grsp.objectWithCover.object.fkine(u));

    % for each contact model...
    %
    % compute the corresponding contribute to the coord transf matrix
    % (txi = Ad_g_cio*Ad_g_ba, Tx = diag(txi))
    txi = cellfun(@(x) llcs_iad2(x)*Ad_g_ba, ...
                            {grsp.contactStructure.contactModels.contactFrameObjectSide}, ...
                                'UniformOutput', false);

    Tx = blkdiag(txi{:});

    % contact Jacobian
    Jtilde = Tx*Jtilde_s;

    % eventually reduce to 2D
    if reducedProb.isSet,
        Jtilde = reducedProb.reduceDiffMotionsMapTo2d(Jtilde);
    end