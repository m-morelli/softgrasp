%GrOutRigidMobility.AFME. Compute the Active Force Manipulability Ellipsoid for Rigid Grasps Without Synergies
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

function [V, D, tw, ttt] = AFME(grOutRM, ww_, wtt_)

    % checking rhs args
    %
    % right number of rhs args?
    if nargin < 1,
        error('GRASP:classProps:NonConsistentValue', ...
            'wrong number of rhs arguments, expected at least a GrOutRigidManipulability object');
    end
    % check if grOutRM is a Grasp
    if ~isa(grOutRM, 'GrOutRigidManipulability'),
        error('GRASP:classProps:UnknownInput', ...
            'wrong input argument, expected a GrOutRigidManipulability object');
    end
    % check for optional argument
    ww = eye(grOutRM.nw);
    wtt = eye(grOutRM.na);
    if nargin > 1,
        if ~isa(ww_, 'numeric'),
            error('GRASP:classProps:NonConsistentValue', ...
                'wrong value specified for object-space velocity weights ''Ww'', expected a numeric matrix');
        end
        if or(size(ww_) ~= [grOutRM.nw, grOutRM.nw]),
            error('GRASP:classProps:NonConsistentValue', ...
                ['wrong value specified for object-space velocity weights ''Ww'', expected a square matrix of order ', mat2str(grOutRM.nw)]);
        end
        ww = ww_;
        if nargin > 2,
            if ~isa(wtt_, 'numeric'),
                error('GRASP:classProps:NonConsistentValue', ...
                    'wrong value specified for joint-space velocity weights ''Wtau'', expected a numeric matrix');
            end
            if or(size(wtt_) ~= [grOutRM.na, grOutRM.na]),
                error('GRASP:classProps:NonConsistentValue', ...
                    ['wrong value specified for joint-space velocity weights ''Wtau'', expected a square matrix of order ', mat2str(grOutRM.na)]);
            end
            wtt = wtt_;
        end
    end

    % Gamma_w, Gamma_s, Gamma_t, Gamma_h
    gmw = grOutRM.gmw;
    gms = grOutRM.gms;
    gmt = grOutRM.gmt;
    gmh = grOutRM.gmh;

    % compute the Projector matrix
    if ~(isempty(gms) || all(all(gms == 0,2))),
        gmw = llcs_projector_matrix(gms, ww)*gmw;
    end

    % setting up the eigenvalue problem of active force manipulability
    num = blkdiag(gmw'*ww*gmw, zeros(grOutRM.nh, grOutRM.nh));
    den = blkdiag(gmt'*wtt*gmt, gmh'*wtt*gmh);
    tw = [gmw, zeros(grOutRM.nw, grOutRM.nh)];
    ttt = [gmt, gmh];
    [V,D] = eig(num, den);