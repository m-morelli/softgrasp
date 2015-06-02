% GraspAnalysisSolver.ADDSOFTSYNERGY. Add Soft Synergistic Actuation to a Robotic Hand
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

function addSoftSynergy(grsp_poe, S, Kq)


    % adds to the last level the Soft Synergies equations:
    %
    %   eps -Kq*qr +Kq*q = 0   <--- equilibrium equation
    %
    %   q -S*s = 0
    %
    %   eps -S'*tau = 0  <--- equilibrium equation
    %

    if (size(S,2)>size(S,1))
        error('TRG:addSoftSynergy:NonConsistentValue', 'The number of Synergies must be lower or equal than the joint parameters.')
    end

    % it handles Kq in both cases: passing the whole matrix to the function, or
    % passing only the diagonal elements.
    if isvector(Kq)
        Kq = diag(Kq);
    else
        if ( size(Kq,1) ~= size(Kq,2) )
        error('TRG:addSoftSynergy:NonConsistentValue', 'Kq must be square');
        end
    end

    qd = size(Kq,1);    % size of joint parameters
    zd = size(S,2);     % size of synergies

    dd = grsp_poe.FGMstruct.dim(grsp_poe.FGMstruct.pointers(2));

    if(size(S,1)==dd)
        if(size(Kq,1)==dd)
            grsp_poe.FGMstruct.levels = grsp_poe.FGMstruct.levels + 1;

            % check the FGM dimensions, to correctly build the additional parts
            dim = size(grsp_poe.FGMstruct.FGM);
            A_inf = zeros(qd+qd+zd,dim(2));
            A_vd = zeros(dim(1),qd+zd+zd);

            c1 = 1;
            for i = 1 : grsp_poe.FGMstruct.pointers(1)-1
                c1 = c1 + grsp_poe.FGMstruct.dim(i);
            end
            c2 = 1;
            for i = 1 : grsp_poe.FGMstruct.pointers(2)-1
                c2 = c2 + grsp_poe.FGMstruct.dim(i);
            end

            A_inf(1:qd,c1:c1+qd-1) = eye(qd);    
            A_inf(1:qd, c2:c2+qd-1) = Kq;      

            A_inf(qd+1:qd+zd,c1:c1+qd-1) = -S';

            A_inf(qd+zd+1:qd+zd+qd,c2:c2+qd-1) = eye(qd);


            grsp_poe.FGMstruct.FGM = [  grsp_poe.FGMstruct.FGM            A_vd; 
                                                    A_inf       blkdiag(-Kq,eye(zd),-S)];

            % adding qr, epsi, s to the configuration
            grsp_poe.FGMstruct = grsp_poe.fgmStructAddConf({strcat('d_qr_',int2str(grsp_poe.FGMstruct.levels)),...
                                                            strcat('d_eps_',int2str(grsp_poe.FGMstruct.levels)),...
                                                            strcat('d_s_',int2str(grsp_poe.FGMstruct.levels))},[qd;zd;zd]); % <-- not equilibrium (last row)

            grsp_poe.FGMstruct.num_equations = grsp_poe.FGMstruct.num_equations + 3;

            % updating the general force and displacement pointers
            n = size(grsp_poe.FGMstruct.conf,1);
            grsp_poe.FGMstruct.pointers = [n-1;n];
        else
            error('TRG:addSoftSynergy:NonConsistentValue', 'Wrong size for matrix Kq');
        end

    else
        error('TRG:addSoftSynergy:NonConsistentValue', 'Wrong size for matrix S');
    end

    display(grsp_poe);

end