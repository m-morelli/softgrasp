%Limb.JACOB0. Evaluates the spatial Jacobian for a Limb wrt the base reference frame
%   Description, in progress
%
%   Annotations, in progress
%
%   References, in progress
%
%   Nicola Greco, I.R.C. "E. Piaggio", University of Pisa
%   Matteo Morelli, I.R.C. "E. Piaggio", University of Pisa
%
%   March 2013

% Copyright (C) 2013 Interdepartmental Research Center "E. Piaggio", University of Pisa
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

function J_chain_i = jacob0(limb, q)

    J_chain_i = [];

    % number of inputs must be >=2 and <=3 (TODO: conf?)
    narginchk(2, 2);

    % check if limb is a Limb
    if ~isa(limb, 'Limb'),
        error('TRG:jacob0:UnknownInput', 'Wrong input argument, expected a Limb object.');
    end

    % if q is numeric
    if ~isa(q, 'numeric'),
        error('TRG:jacob0:UnknownProperty', 'Wrong input argument, expected a configuration vector (q).');
    end
    q = real(q(:));

    % check conf size
    if numel(q) ~= limb.descr.n,
        error('TRG:jacob0:NonConsistentValue', 'Wrong joint config, expected %d elements in place of %d.', limb.descr.n, numel(q));
    end

    % prepare for computation
    Z = limb.descr.Z;
    P = limb.descr.P;
    joints = limb.descr.joints;
    n = limb.descr.n;

    % do job
    for i = 1:n,
        c = joints(i);
        switch(c),	% switch between prismatic or revolute joint
            case 0,  % revolute joint
                if(i == 1)  % evaluate xi
                    xi_i = [-cross(Z(:,i),P(:,i)) ; Z(:,i)];
                    J_chain_i = xi_i;
                    xi_next = llcs_twistexp2(xi_i,q(i));
                else  %  evaluate xi'
                    xi_i = [-cross(Z(:,i),P(:,i)) ; Z(:,i)];
                    J_chain_i = [J_chain_i llcs_ad2(xi_next)*xi_i];
                    xi_next = xi_next*llcs_twistexp2(xi_i,q(i));
                end
            case 1,  % prismatic joint
                if(i == 1)  % evaluate xi
                    xi_i = [Z(:,i); zeros(size(Z(:,i),1),1)];
                    J_chain_i = [xi_i];
                    xi_next = llcs_twistexp2(xi_i,q(i));
                else  %  evaluate xi'
                    xi_i = [Z(:,i); zeros(size(Z(:,i),1),1)];
                    J_chain_i = [J_chain_i llcs_ad2(xi_next)*xi_i];
                    xi_next = xi_next*llcs_twistexp2(xi_i,q(i));
                end
        end
    end

end