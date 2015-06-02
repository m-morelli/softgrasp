%GrOutRigidMobility.KME. Compute the Kinematic Manipulability Ellipsoid for Rigid Grasps Without Synergies
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

function [V, D, tu, tq] = KME(grOutRM, wu_, wq_)

    % checking rhs args
    %
    % right number of rhs args?
    if nargin < 1,
        error('GRASP:classProps:NonConsistentValue', ...
            'wrong number of rhs arguments, expected at least a GrOutRigidMobility object');
    end
    % check if grOutRM is a Grasp
    if ~isa(grOutRM, 'GrOutRigidMobility'),
        error('GRASP:classProps:UnknownInput', ...
            'wrong input argument, expected a GrOutRigidMobility object');
    end
    % check for optional argument
    wu = eye(grOutRM.nu);
    wq = eye(grOutRM.na);
    if nargin > 1,
        if ~isa(wu_, 'numeric'),
            error('GRASP:classProps:NonConsistentValue', ...
                'wrong value specified for object-space velocity weights ''Wu'', expected a numeric matrix');
        end
        if or(size(wu_) ~= [grOutRM.nu, grOutRM.nu]),
            error('GRASP:classProps:NonConsistentValue', ...
                ['wrong value specified for object-space velocity weights ''Wu'', expected a square matrix of order ', mat2str(grOutRM.nu)]);
        end
        wu = wu_;
        if nargin > 2,
            if ~isa(wq_, 'numeric'),
                error('GRASP:classProps:NonConsistentValue', ...
                    'wrong value specified for joint-space velocity weights ''Wq'', expected a numeric matrix');
            end
            if or(size(wq_) ~= [grOutRM.na, grOutRM.na]),
                error('GRASP:classProps:NonConsistentValue', ...
                    ['wrong value specified for joint-space velocity weights ''Wq'', expected a square matrix of order ', mat2str(grOutRM.na)]);
            end
            wq = wq_;
        end
    end
    
    % Gamma_r, Gamma_qc, Gamma_oc, Gamma_ii
    gmr = [grOutRM.gmar, grOutRM.gmapc];
    gmqc = [grOutRM.gmapoc, grOutRM.gmaoc];
    gmoc = [grOutRM.gmopac, grOutRM.gmoac];
    gmii = [grOutRM.gmopi, grOutRM.gmoi];

    % compute the Projector matrix
    if ~(isempty(gmqc) || all(all(gmqc == 0,2))),
        if ~(isempty(gmii) || all(all(gmii == 0,2))),
            % BP2000 (13)
            gmoc = llcs_projector_matrix(gmii, wu)*gmoc;
        end
        if ~(isempty(gmr) || all(all(gmr == 0,2))),
            gmqc = llcs_projector_matrix(gmr, wq)*gmqc;
        end
    else
        return; % TODO
    end

    % the eigenvalue problem of manipulability ratio for kinematically
    % redundant and indeterminate mechanisms
    num = gmoc'*wu*gmoc;
    den = gmqc'*wq*gmqc;
    tu = gmoc;
    tq = gmqc;
    [V,D] = eig(num, den);