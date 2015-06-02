%GrOutRigidMobility.IFME. Compute the Internal Force Manipulability Ellipsoid for Rigid Grasps Without Synergies
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

function [V, D, tth, ttth] = IFME(grOutRM, wth_, wtth_)

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
    fhr = size(grOutRM.gmfh,1);
    wth = eye(fhr);
    wtth = eye(grOutRM.na);
    if nargin > 1,
        if ~isa(wth_, 'numeric'),
            error('GRASP:classProps:NonConsistentValue', ...
                'wrong value specified for object-space velocity weights ''Wth'', expected a numeric matrix');
        end
        if or(size(wth_) ~= [fhr, fhr]),
            error('GRASP:classProps:NonConsistentValue', ...
                ['wrong value specified for object-space velocity weights ''Wth'', expected a square matrix of order ', mat2str(fhr)]);
        end
        wth = wth_;
        if nargin > 2,
            if ~isa(wtth_, 'numeric'),
                error('GRASP:classProps:NonConsistentValue', ...
                    'wrong value specified for joint-space velocity weights ''Wtauh'', expected a numeric matrix');
            end
            if or(size(wtth_) ~= [grOutRM.na, grOutRM.na]),
                error('GRASP:classProps:NonConsistentValue', ...
                    ['wrong value specified for joint-space velocity weights ''Wtauh'', expected a square matrix of order ', mat2str(grOutRM.na)]);
            end
            wtth = wtth_;
        end
    end

    % Gamma_fh, Gamma_h
    gmfh = grOutRM.gmfh;
    gmh = grOutRM.gmh;

    % early return when Gamma_fh, Gamma_h are empty
    if isempty(gmfh) && isempty(gmh),
        V = [];
        D = V;
        tth = V;
        ttth = V;
        return;
    end

    % the eigenvalue problem for the internal forces manipulability
    num = gmfh'*wth*gmfh;
    den = gmh'*wtth*gmh;
    tth = gmfh;
    ttth = gmh;
    [V,D] = eig(num, den);