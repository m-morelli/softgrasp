%LimbDescriptionDH.toScrew. Convert a RTB SerialLInk model to an equivalent Screw representation
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

function toScrew(objScrew, objDH)

    % number of inputs must be exactly 1 (plus another one for "this" object)
    narginchk(2, 2);

    % check if objDH is a RTB SerialLink object
    if ~isa(objDH, 'SerialLink'),
        error('TRG:toScrew:UnknownInput', 'Wrong input argument, expected a SerialLink object.');
    end

    % prepare for computation
    T_i = objDH.base;
    dofs = objDH.n;
    nJointTypes = [0,1];

    % standard or modified DH?
    if ~objDH.mdh,
        % standard
        P = [T_i((1:3),4), zeros(3, dofs-1)];
        Z = [T_i((1:3),3), zeros(3, dofs-1)];
        % do job
        for i = 1:dofs-1,
            %T_i = A_i(:,:,i)*T_i; ??
            T_i = T_i*objDH.links(i).A(0);
            P(:,i+1) = T_i((1:3),4);
            Z(:,i+1) = T_i((1:3),3);
        end
        g_st0 = T_i*objDH.links(dofs).A(0)*objDH.tool;

    else
        % modified
        ...
    end

    % ok now build the Screw limb represent.
    objScrew.n = dofs;
    objScrew.P = P;
    objScrew.Z = Z;
    objScrew.g_st0 = g_st0;
    objScrew.joints = nJointTypes((objDH.config=='P')+1);

end