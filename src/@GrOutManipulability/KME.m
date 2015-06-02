%GrOutManipulability.KME. Compute the Kinematic Manipulability Ellipsoid for Compliant Grasps Wuth Postural Synergies
%   Description, in progress
%
%   Annotations, in progress
%
%   References, in progress
%
%   Matteo Morelli, I.R.C. "E. Piaggio", University of Pisa
%
%   June 2012

% Copyright (C) 2012 Interdepartmental Research Center "E. Piaggio", University of Pisa
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

function [V, D, tu, tz] = KME(grOutSM, wu_, wz_)

    % checking rhs args
    %
    % right number of rhs args?
    if nargin < 1,
        error('GRASP:classProps:NonConsistentValue', ...
            'wrong number of rhs arguments, expected at least a GrOutManipulability object');
    end
    % check if grOutRM is a Grasp
    if ~isa(grOutSM, 'GrOutManipulability'),
        error('GRASP:classProps:UnknownInput', ...
            'wrong input argument, expected a GrOutManipulability object');
    end
    % check for optional argument
    wu = eye(grOutSM.nd);
    wz = eye(grOutSM.nz);
    if nargin > 1,
        if ~isa(wu_, 'numeric'),
            error('GRASP:classProps:NonConsistentValue', ...
                'wrong value specified for object-space velocity weights ''Wu'', expected a numeric matrix');
        end
        if or(size(wu_) ~= [grOutSM.nd, grOutSM.nd]),
            error('GRASP:classProps:NonConsistentValue', ...
                ['wrong value specified for object-space velocity weights ''Wu'', expected a square matrix of order ', mat2str(grOutSM.nd)]);
        end
        wu = wu_;
        if nargin > 2,
            if ~isa(wz_, 'numeric'),
                error('GRASP:classProps:NonConsistentValue', ...
                    'wrong value specified for synergy-space velocity weights ''Wz'', expected a numeric matrix');
            end
            if or(size(wz_) ~= [grOutSM.nz, grOutSM.nz]),
                error('GRASP:classProps:NonConsistentValue', ...
                    ['wrong value specified for synergy-space velocity weights ''Wz'', expected a square matrix of order ', mat2str(grOutSM.nz)]);
            end
            wz = wz_;
        end
    end
    
    % Gamma_uk, Gamma_zk, Gamma_zrr, Gamma_uri
    gmuk = [grOutSM.gmurc, grOutSM.gmuh, grOutSM.gmuc, grOutSM.gmus];
    gmzk = [grOutSM.gmzrc, grOutSM.gmzh, grOutSM.gmzc, grOutSM.gmzs];
    gmzrr = grOutSM.gmzrr;
    gmuri = grOutSM.gmuri;

    % compute the Projector matrix
    if ~(isempty(gmzk) || all(all(gmzk == 0,2))),
        if ~(isempty(gmuri) || all(all(gmuri == 0,2))),
            % P.etAllP2011 (25)
            gmuk = llcs_projector_matrix(gmuri, wu)*gmuk;
        end
        if ~(isempty(gmzrr) || all(all(gmzrr == 0,2))),
            gmzk = llcs_projector_matrix(gmzrr, wz)*gmzk;
        end
    else
        return; % TODO
    end

    % the eigenvalue problem of manipulability ratio for kinematically
    % redundant and indeterminate mechanisms
    num = gmuk'*wu*gmuk;
    den = gmzk'*wz*gmzk;
    tu = gmuk;
    tz = gmzk;
    [V,D] = eig(num, den);