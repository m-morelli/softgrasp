% GraspAnalysisSolver.NULLSPACEDECOMP. Fundamental Routines for FGM's NullSpace Decomposition
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

function structOut = nullSpaceDecomp(grsp_poe, set_zero, not_zero)

    sz = size(set_zero,2); % number of elements to be set to zero
    nz = size(not_zero,2); % number of elements not to be set to zero
    k = size(grsp_poe.FGMstruct.conf,1);  % configuration vector size
    for i = 1 : sz
        [~,~,~,flag] = grsp_poe.fgmStructFindConf(set_zero{i});
        if(~flag)
            return
        end
    end
    for i = 1 : nz
        [~,~,~,flag] = grsp_poe.fgmStructFindConf(not_zero{i});
        if(~flag)
            return
        end
    end

    E = null(grsp_poe.FGMstruct.FGM, 'r');
    %E = null(grsp_poe.FGMstruct.FGM);

    for i = 1 : sz
        if(isempty(E))
            structOut = [];
            return
        end

        E = srv_decomposition(grsp_poe,E,set_zero{i},'zero');
    end

    for i = 1 : nz
        if(isempty(E))
            structOut = [];
            return
        end
        E = srv_decomposition(grsp_poe,E,not_zero{i},'notzero');

    end

    p = 1;
    for i = 1 : k
        eval(['structOut.',grsp_poe.FGMstruct.conf{i},'=E(p:p+grsp_poe.FGMstruct.dim(i)-1,:);']);
        p = p + grsp_poe.FGMstruct.dim(i);
    end
end

% service routine for Nullspace decomposition;
% Inputs:
% A = structure collecting FGM, configuration and dimensions
% E = nullspace of FGM
% elem = string which represents the element to be isolated
% mode = zero/notzero, choose between selecting the zero or the non zero
% columns from the nullspace
function T1 = srv_decomposition(grsp_poe,E,elem,mode)

    T1_=[];
    T = [];
    T2_ = [];

    k = size(grsp_poe.FGMstruct.conf,1);  % configuration vector size

    from = 1; % start index
    to = 0;
    flag = 1;

    for i = 1 : k  % for each element in the configuration, it extracts the relative nullspace rows
        to = to + grsp_poe.FGMstruct.dim(i);

        E_tmp = E(from:to,:);

        if(strcmp(grsp_poe.FGMstruct.conf{i},elem))
            T = E_tmp;
            flag = 2;
        else
            switch(flag)

                case 1
                    T1_ = [T1_;
                           E_tmp;];
                case 2
                    T2_ = [T2_;
                           E_tmp;];
            end
        end

    from = to + 1;  % update the from pointer
    end


    [nr,nc] = size(T);

    % disp([T' eye(nc)]); % display the structur sent to the rref routine

    %disp('RREF form:');
    RREF_form = rref([T' eye(nc)]);

    %disp('Permutation Matrix:');
    P = RREF_form(:,end-nc+1:end);  % extracts the permutation matrix

    %disp('U:');
    U = RREF_form(:,1:nr);  % extracts the U component(including zeros)

    %disp('gamma_one:');% New form for the nullspace basis, with "dirty" rows
    if(size(T1_,1) == 0)
        T1 = [     U'   ;
                (P*(T2_'))'];

    else
        if(size(T2_,1) == 0)
            T1 = [ (P*(T1_'))';
                        U'   ];
        else
            T1 = [ (P*(T1_'))';
                    U'    ;
               (P*(T2_'))'];
        end
    end

    switch(mode)
        case 'zero'
            vect = find_null(U');
        case 'notzero'
            vect = find_no_null(U');
    end

    %disp(vect);

    if(isempty(vect))
        disp('Configuration unreachable');
        Tnew = [];
    else
        % if the configuration is reachable, creates a data structure
        % containing all the basis divided for each configuration element
        k = size(vect,2); 
        Tnew = [];
        for i = 1 : k
            Tnew =[Tnew T1(:,vect(i))]; %#ok<*AGROW>
        end
    end
    T1 = Tnew;
end

function vect = find_null(U)
    vect = [];
    for j = 1 : size(U,2)
        if(norm(U(:,j)) == 0)
            vect = [vect j];
        end
    end 
end

function vect = find_no_null(U)
    vect = [];
    for j = 1 : size(U,2)
        if(norm(U(:,j)) ~= 0)
            vect = [vect j];
        end
    end
end