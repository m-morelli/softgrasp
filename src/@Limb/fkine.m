%Limb.FKINE. Evaluates Forward Kinematics of a Limb Model for a Given Joint-Set
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

function T = fkine(limb, q)

    T = [];

    % check for exactly 2 input args
    narginchk(2, 2);

    % check if limb is a Limb
    if ~isa(limb, 'Limb'),
        error('TRG:fkine:UnknownInput', 'Wrong input argument, expected a Limb object.');
    end

    % if q is numeric
    if ~isa(q, 'numeric'),
        error('TRG:fkine:UnknownProperty', 'Wrong input argument, expected a configuration vector (q).');
    end
    q = real(q(:));

    % check conf size
    if numel(q) ~= limb.descr.n,
        error('TRG:fkine:NonConsistentValue', 'Wrong joint config, expected %d elements in place of %d.', limb.descr.n, numel(q));
    end

    % prepare for computation
    Z = limb.descr.Z;
    P = limb.descr.P;
    joints = limb.descr.joints;

    % do job
    for i = 1 : limb.descr.n,

        if(joints(i) == 1) % prismatic
            xi_i = [Z(:,i); zeros(size(Z(:,i),1),1)];   % twist

        else % revolute
            xi_i = [-cross(Z(:,i),P(:,i)) ; Z(:,i)];    % twist

        end
        g_i = llcs_twistexp2(xi_i, q(i));
        if(i == 1)
            T = g_i;
        else
            T = T*g_i;
        end

    end

    T = T*limb.descr.g_st0;
end