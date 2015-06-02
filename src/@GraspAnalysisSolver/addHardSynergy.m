% GraspAnalysisSolver.ADDHARDSYNERGY. Add Hard Synergistic Actuation to a Robotic Hand
%   Description, in progress
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

function addHardSynergy(grsp_poe, S)

    %  adds to the last level the two Hard Synergies equations:
    %
    %   q -S*s = 0   
    %
    %   eps -S'*tau = 0   <-- equilibrium equation
    %

    if (size(S,2)>size(S,1))
        error('TRG:addHardSynergy:NonConsistentValue', 'The number of Synergies must be lower or equal than the joint numbers.')
    end

    dd = grsp_poe.FGMstruct.dim(grsp_poe.FGMstruct.pointers(2)); % pointer to the general displacement

    if(size(S,1) == dd)

        zd = size(S,2);
        qd = size(S,1);
        grsp_poe.FGMstruct.levels = grsp_poe.FGMstruct.levels + 1;

        % check the FGM dimensions, to correctly build the additional parts
        dim = size(grsp_poe.FGMstruct.FGM);
        A_inf = zeros(qd+zd,dim(2));
        A_vd = zeros(dim(1),zd+zd);

        c1 = 1;
        for i = 1 : grsp_poe.FGMstruct.pointers(1)-1
            c1 = c1 + grsp_poe.FGMstruct.dim(i);
        end

        c2 = 1;
        for i = 1 : grsp_poe.FGMstruct.pointers(2)-1
            c2 = c2 + grsp_poe.FGMstruct.dim(i);
        end

        A_inf(1:zd,c1:c1+qd-1) = -S';

        A_inf(zd+1:zd+qd,c2:c2+qd-1) = eye(qd);

        grsp_poe.FGMstruct.FGM = [  grsp_poe.FGMstruct.FGM            A_vd; 
                                              A_inf           blkdiag(eye(zd),-S)];

        % adding epsi & s to the configuration    
        grsp_poe.FGMstruct = grsp_poe.fgmStructAddConf({strcat('d_eps_',int2str(grsp_poe.FGMstruct.levels)),...
                                                        strcat('d_s_',int2str(grsp_poe.FGMstruct.levels))},[zd;zd]);

        % updating the general force and displacement pointers
        n = size(grsp_poe.FGMstruct.conf,1);
        grsp_poe.FGMstruct.pointers = [n-1;n];

        grsp_poe.FGMstruct.num_equations = grsp_poe.FGMstruct.num_equations + 2;


    else
        error('TRG:addHardSynergy:NonConsistentValue', 'Wrong domensions for S: it must be %d x num_syn', dd);
    end

    display(grsp_poe);
end