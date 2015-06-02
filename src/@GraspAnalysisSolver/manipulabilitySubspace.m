%Grasp.MANIPULABILITYSUBSPACE. Compute the Manipolability Subspaces for Compliant Grasps With Postural Synergies
%   Description, in progress
%
%   Annotations, in progress
%
%   References, in progress
%
%   Matteo Morelli, I.R.C. "E. Piaggio", University of Pisa
%
%   May 2012
%   Modified on August 2013
%       - the file is now part of @GraspAnalysisSolver
%       - aligned with the computations described in [1]

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

function [gOutSM] = manipulabilitySubspace(grsp)

    % checking rhs args
    %
    % right number of rhs args?
    narginchk(1,1);
    % check if grsp is a GraspAnalysisSolver
    if ~isa(grsp, 'GraspAnalysisSolver'),
        error('TRG:manipulabilitySubspace:NonConsistentValue', ...
            'Wrong input argument, expected a GraspAnalysisSolver object.');
    end
    % reduced prob?
    %reducedProb = grsp.reducedProb;

    % compute the internal perturbs (i)
    %
    % internal complementary (1)
    ic_m = grsp.methodOutStruct2Mat(grsp.internalCompl);
    % spurious squeeze (3)
    ss_m = grsp.methodOutStruct2Mat(grsp.spuriousSqueeze);
    % redundant motions (5)
    rm_m = grsp.methodOutStruct2Mat(grsp.redundantMotions);
    % gm_i (matrix)
    gm_i_m = [ic_m, ss_m, rm_m];
    % convert gm_i matrix to gm_i struct
    gm_i_s = [];
    if ~isempty(gm_i_m),
        gm_i_s = grsp.methodOutMat2Struct(gm_i_m);
    end

    % compute the pure squeezes (sq)
    %
    gm_sq_s = grsp.pureSqueeze;
    gm_sq_m = grsp.methodOutStruct2Mat(gm_sq_s);

    % compute the kinem. displacements (k)
    %
    gm_k_s = grsp.kineGraspDispl;
    gm_k_m = grsp.methodOutStruct2Mat(gm_k_s);

    % compute the external structural forces (st)
    %
    gm_st_s = grsp.extStructForces;
    gm_st_m = grsp.methodOutStruct2Mat(gm_st_s);

    % compute the coordinated perturbs (co)
    %
    gm_co_s = grsp.coordinatePerturbs;
    gm_co_m = grsp.methodOutStruct2Mat(gm_co_s);

    gOutSM = GrOutManipulability(...
                'FGMstruct',  grsp.FGMstruct, ...
                'ni',         size(gm_i_m,2), ...
                'nsq',        size(gm_sq_m,2), ...
                'nk',         size(gm_k_m,2), ...
                'nst',        size(gm_st_m,2), ...
                'nco',        size(gm_co_m,2), ...
                'gm_i',       gm_i_s, ...
                'gm_sq',      gm_sq_s, ...
                'gm_k',       gm_k_s, ...
                'gm_st',      gm_st_s, ...
                'gm_co',      gm_co_s);