%ClosedChainsSolver.SUBSPACEDECOMP. Core Function For Closed-Chains' Mobility/Manipolability Subspaces Decomposition
%   Description, in progress
%
%   Annotations, in progress
%
%   References, in progress
%
%   Matteo Morelli, I.R.C. "E. Piaggio", University of Pisa
%
%   May 2012
%   Modified on July 2013
%       - Moved to @ClosedChainsSolver/
%       - Using @FormulationAdapter/
%       - Err. check.

% Copyright (C) 2012, 2013 Interdepartmental Research Center "E. Piaggio", University of Pisa
%
% This file is part of THE Robotic Grasping Toolbox for use with MATLAB(R).
%
% THE Robotic Grasping Toolbox for use with MATLAB(R) is free software: you can redistribute it and/or modify it under
% the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the
% License, or (at your option) any later version.
%
% THE Robotic Grasping Toolbox for use with MATLAB(R) is distributed in the hope that it will be useful, but WITHOUT ANY
% WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
% General Public License for more details.
%
% You should have received a copy of the GNU General Public License along with THE Robotic Grasping Toolbox for use with
% MATLAB(R).  If not, see <http://www.gnu.org/licenses/>.

function [gOutRM] = subspaceDecomp(ccs, q, u, sdCbk)

    % checking rhs args
    %
    % right number of rhs args?
    narginchk(4,4);
    % check if grsp is a ClosedChainsSolver
    if ~isa(ccs, 'ClosedChainsSolver'),
        error('TRG:rigidMobilitySubspace:NonConsistentValue', ...
            'Wrong input argument, expected a ClosedChainsSolver object.');
    end
    if ~isa(q, 'numeric'),
        error('TRG:rigidMobilitySubspace:NonConsistentValue', ...
            'Wrong input argument, expected a configuration vector (q).');
    end
    q = real(q(:));
    if ~isa(u, 'numeric'),
        error('TRG:rigidMobilitySubspace:NonConsistentValue', ...
            'Wrong input argument, expected an object configuration vector (u).');
    end
    u = real(u(:));
    % check the callback
    if ~isa(sdCbk, 'function_handle'),
        error('TRG:subspaceDecomp:UnknownMethod', ...
            'Wrong input argument, expected a callback for manipulability/mobility subspace decomposition (ClosedChainsSolver).');
    end
    % reducedProb?
    reducedProb = ccs.reducedProb;

    % collect useful data struct/info
    nc = ccs.contactStructure.nContactModels;

    % compute the Complete Grasp Matrix
    [Tlx, Trx] = FormulationAdapter.graspMatrixAdapter(ccs.objectWithCover, u, nc);
    Gtilde = Tlx*ccs.completeGraspMatrix*Trx;

    % compute the Complete Jacobian Matrix
    Ux = FormulationAdapter.jacobianMatrixAdapter(ccs.handWithGlove, q, nc);
    Jtilde = Ux*ccs.completeHandJacobianSpatial(q);

    % selection matrix
    H = FormulationAdapter.selectionMatrix(ccs.contactStructure.contactModels, u, reducedProb);

    % permutation matrix
    [Pi, na] = ccs.passiveJointsPermutationMatrix;

    % adjust Jacobian Matrix
    JXtilde = Jtilde*Pi;
    Ja = H*JXtilde(:,1:na); Jp = H*JXtilde(:,na+1:end);

    % perform the real computation
    [gOutRM] = sdCbk(Gtilde*H', Ja, Jp, 1e-8);