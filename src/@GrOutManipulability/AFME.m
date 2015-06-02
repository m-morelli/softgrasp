%GrOutManipulability.AFME. Compute the Active Force Manipulability Ellipsoid for Compliant Grasps Wuth Postural
%Synergies
%   Description, in progress
%
%   Annotations, in progress
%
%   References, in progress
%
%   Matteo Morelli, I.R.C. "E. Piaggio", University of Pisa
%
%   June 2012
%   Modified on August 2013

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

function [V, D, tw, ts] = AFME(grom, ww_, ws_)

    % checks
    %
    % right number of rhs args?
    narginchk(1,3);

    % check if grOutRM is a Grasp
    if ~isa(grom, 'GrOutManipulability'),
        error('TRG:AFME:NonConsistentValue', ...
            'Wrong input argument, expected a GrOutManipulability object.');
    end

    % extract fundamental info from the FGMstruct
    fgms = grom.FGMstruct;
    nd = fgms.dim( strcmp('d_we', fgms.conf) );   % dim(we)
    nz = fgms.dim( fgms.pointers(2) );            % dim(last-level gen.displ)
    lab_llfgen = fgms.conf{ fgms.pointers(1) };   % label of last-level generalized forces

    % checks (cont'd)
    %
    % check for optional argument
    ww = eye(nd);
    ws = eye(nz);
    if nargin > 1,
        if ~isa(ww_, 'numeric'),
            error('TRG:AFME:NonConsistentValue', ...
                'Wrong value specified for object-space velocity weights ''W_w'', expected a numeric matrix.');
        end
        if or(size(ww_) ~= [nd, nd]),
            error('TRG:AFME:NonConsistentValue', ...
                'Wrong value specified for object-space velocity weights ''W_w'', expected a square matrix of order %d.', ...
                    nd);
        end
        ww = ww_;
        if nargin > 2,
            if ~isa(ws_, 'numeric'),
                error('TRG:AFME:NonConsistentValue', ...
                    'Wrong value specified for control-space velocity weights ''W_eta'', expected a numeric matrix.');
            end
            if or(size(ws_) ~= [nz, nz]),
                error('TRG:AFME:NonConsistentValue', ...
                    'Wrong value specified for control-space velocity weights ''W_eta'', expected a square matrix of order %d.', ...
                        nz);
            end
            ws = ws_;
        end
    end

    % computs.
    %
    % Gamma_{w_a}, Gamma_{eta_a}
    gm_w_co = grom.gm_co.d_we;
    gm_w_a = [zeros(nd, grom.ni+grom.nsq), gm_w_co];
    gm_eta_i = grom.gm_i.(lab_llfgen);
    gm_eta_sq = grom.gm_sq.(lab_llfgen);
    gm_eta_co = grom.gm_co.(lab_llfgen);
    gm_eta_a = [gm_eta_i, gm_eta_sq, gm_eta_co];
    %
    % Projector matrix
    gm_w_st = grom.gm_st.d_we;
    if ~(isempty(gm_w_st) || all(all(gm_w_st == 0,2))),
        gm_w_a = llcs_projector_matrix(gm_w_st, ww)*gm_w_a;
    end
    %
    % setting up the eigenvalue problem of active force manipulability
    num = gm_w_a'*ww*gm_w_a;
    den = gm_eta_a'*ws*gm_eta_a;
    tw = gm_w_a;
    ts = gm_eta_a;
    [V,D] = eig(num, den);