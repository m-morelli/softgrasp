% GraspAnalysisSolver.ADDCOMPLIANTJOINT. Add Compliant Joint Actuation to (Synergistic Underactuated) Robotic Hands
%   The function adds the Compliant Joint Actuation to the hand
%   configuration. By choosing a suitable input matrix (Kz or Kq) 
%   it is possible to actuate either synergies or simple joints.
%
%   Annotations, in progress
%
%   References, in progress
%
%   Nicola Greco, I.R.C. "E. Piaggio", University of Pisa
%   Edoardo Farnioli, I.R.C. "E. Piaggio", University of Pisa
%
%   July 2013

% Copyright (C) 2013 Interdepartmental Research Center "E. Piaggio", University of Pisa
%
% This file is part of THE Robotic Grasping Toolbox for use with MATLAB(R).
%
% THE Robotic Grasping Toolbox for use with MATLAB(R) is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
%
% THE Robotic Grasping Toolbox for use with MATLAB(R) is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
%
% You should have received a copy of the GNU General Public License
% along with THE Robotic Grasping Toolbox for use with MATLAB(R).  If not, see <http://www.gnu.org/licenses/>.

function addCompliantJoints(grsp_poe, Kdd)


    % adds to the last level the equation:
    %
    %   tau -Kq*qr +Kq*q = 0  <--- equilibrium equation
    %
    % or
    %
    %   eps -Ks*sr +Ks*s = 0  <--- equilibrium equation
    %

    % it handles the stiffness matrix in both cases: passing the whole matrix 
    % to the function, or passing only the diagonal elements.
    if isvector(Kdd)
        Kdd = diag(Kdd);
    else
        if ( size(Kdd,1) ~= size(Kdd,2) )
            error('TRG:addCompliantJoints:NonConsistentValue', 'The stiffness matrix must be square.');
        end
    end

    % reads the size of the problem from the general displacement pointer 
    dd = grsp_poe.FGMstruct.dim(grsp_poe.FGMstruct.pointers(2));

    if(size(Kdd,1)==dd)

        grsp_poe.FGMstruct.num_equations = grsp_poe.FGMstruct.num_equations + 1;
        grsp_poe.FGMstruct.levels = grsp_poe.FGMstruct.levels + 1;
        % check the FGM dimensions, to correctly build the additional parts
        dim = size(grsp_poe.FGMstruct.FGM);
        A_inf = zeros(dd,dim(2));
        A_vd = zeros(dim(1),dd);

        c1 = 1;
        c2 = 1;
        for i = 1 : grsp_poe.FGMstruct.pointers(1)-1
            c1 = c1 + grsp_poe.FGMstruct.dim(i);
        end
        for i = 1 : grsp_poe.FGMstruct.pointers(2)-1 
            c2 = c2 + grsp_poe.FGMstruct.dim(i);
        end

        % check the FGM dimensions, to correctly build the additional parts
        A_inf(:,c1:c1+dd-1) = eye(dd);
        A_inf(:, c2:c2+dd-1) = Kdd;

        grsp_poe.FGMstruct.FGM = [ grsp_poe.FGMstruct.FGM     A_vd;
                                            A_inf             -Kdd ];

        % adds the right new element to the configuration (can be z or q
        % reference position)
        s = grsp_poe.FGMstruct.conf{grsp_poe.FGMstruct.pointers(2)};
        grsp_poe.FGMstruct = grsp_poe.fgmStructAddConf({strcat('d_',s(3),'r_',int2str(grsp_poe.FGMstruct.levels))},dd);

        % updating the general displacement pointer
        n = size(grsp_poe.FGMstruct.conf,1);
        grsp_poe.FGMstruct.pointers(2) = n;

    else
        error('TRG:addCompliantJoints:NonConsistentValue', 'Wrong domensions for Kq: it must be %dx%d.', dd, dd);
    end

    display(grsp_poe);
end