% GraspAnalysisSolver. A Collection of Grasp Analysis Tools for Synergistic Underactuated Robotic Hands
%   Description, in progress
%
%   Annotations, in progress
%
%   References, in progress
%
%   Nicola Greco, I.R.C. "E. Piaggio", University of Pisa
%   Edoardo Farnioli, I.R.C. "E. Piaggio", University of Pisa
%   Matteo Morelli, I.R.C. "E. Piaggio", University of Pisa
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

classdef GraspAnalysisSolver < GraspSolver

    properties (Access = protected)
        FGMstruct;
    end
    properties (SetAccess = protected)
        q;
        u;
        preload;
    end
    properties (Dependent)
        FGM;
    end

    methods
        function obj = GraspAnalysisSolver(varargin)
            % collect properties
            [oplGAS, oplOther] = srv_filter_options_from_list({'q','u','preload'}, 3, varargin{:});

            % super() (Solver)
            %
            % set name, comment, GrOpt2dProblem (if any)
            obj@GraspSolver(oplOther{:});

            if nargin > 1,
                % defaults
                handDofs = obj.handWithGlove.hand.dofs;
                q = zeros(handDofs,1);      % default: starting configuration for all joints is 0
                u_dim = obj.pReducedProb.dim;
                u = zeros(u_dim,1);         % default: zero conf for the object
                fc_dim = obj.contactStructure.nTransmittedDofs(obj.pReducedProb);
                preload = zeros(fc_dim,1);  % default: no preload
                % set q, u, preload (if any)
                lOplGAS = length(oplGAS);
                if lOplGAS > 1,
                    i = 1;
                    while i < lOplGAS,
                        processedProp = oplGAS{i};
                        switch processedProp,
                            case 'q',
                                i = i + 1;
                                q = oplGAS{i};
                                if ~isa(q, 'numeric'),
                                    error('TRG:GraspAnalysisSolver:NonConsistentValue', ...
                                        'Wrong input argument, expected a configuration vector (q).');
                                end
                                q = real(q(:));
                                % check conf size
                                if numel(q) ~= handDofs,
                                    error('TRG:GraspAnalysisSolver:NonConsistentValue', ...
                                        'Wrong joint config, expected %d elements in place of %d.', ...
                                            handDofs, numel(q));
                                end
                            case 'u',
                                i = i + 1;
                                u = oplGAS{i};
                                if ~isa(u, 'numeric'),
                                    error('TRG:GraspAnalysisSolver:NonConsistentValue', 'Wrong input argument, expected an object configuration vector (u).');
                                end
                                u = real(u(:));
                                % check conf size
                                if numel(u) ~= u_dim,
                                    error('TRG:GraspAnalysisSolver:NonConsistentValue', ...
                                        'Wrong object config, expected %d elements in place of %d.', ...
                                            u_dim, numel(u));
                                end
                            case 'preload',
                                i = i + 1;
                                preload = oplGAS{i};
                                if ~isa(preload, 'numeric'),
                                    error('TRG:GraspAnalysisSolver:NonConsistentValue', 'Wrong input argument, expected a contact-force vector (fc).');
                                end
                                preload = real(preload(:));
                                % check conf size
                                if numel(preload) ~= fc_dim,
                                    error('TRG:GraspAnalysisSolver:NonConsistentValue', ...
                                            'The number of force/moment components is not consistent with the DoFs transmitted by the contact models, expected %d components instead of %d.', ...
                                                fc_dim, numel(preload));
                                end
                            otherwise,
                                error('TRG:GraspAnalysisSolver:UnknownProperty', ...
                                    'Unknown property passed to GraspAnalysisSolver ''%s''', processedProp);
                        end
                        i = i + 1;
                    end
                end
                obj.q = q;
                obj.u = u;
                obj.preload = preload;

                % FGM struct initialization
                %
                % contact Jacobian
                J_c_tilde = obj.completeHandJacobian(q, u);
                B = obj.selectionMatrix;
                J_c_hand0 = B*J_c_tilde;
                %
                % grasp matrix
                G = obj.completeGraspMatrix*B';
                %
                % Contact stiffness matrix
                Kc = inv(obj.structuralCompliance);
                c = size(B,1);
                %
                % Jacobian's derivatives
                Q = obj.jacobianDerivWrtHandConfig(q, u, preload);
                U = obj.jacobianDerivWrtObjectConfig(q, u, preload);
                %
                % and finally, build the FGM (fully actuated hand)
                obj.FGMstruct.FGM = [ eye(u_dim), zeros(u_dim,handDofs),           G, zeros(u_dim,handDofs), zeros(u_dim,u_dim),    zeros(u_dim,c),    zeros(u_dim,c);...

                           zeros(handDofs,u_dim),         eye(handDofs), -J_c_hand0',                    -Q,                 -U, zeros(handDofs,c), zeros(handDofs,c);...

                                  zeros(c,u_dim),     zeros(c,handDofs),      eye(c),     zeros(c,handDofs),     zeros(c,u_dim),               -Kc,                Kc;...

                                  zeros(c,u_dim),     zeros(c,handDofs),  zeros(c,c),            -J_c_hand0,     zeros(c,u_dim),            eye(c),        zeros(c,c);...

                                  zeros(c,u_dim),     zeros(c,handDofs),  zeros(c,c),     zeros(c,handDofs),                -G',        zeros(c,c),            eye(c)];

                % starting configuration elements and dimensions
                obj.FGMstruct.conf = {'d_we';  'd_tau'; 'd_fc';    'd_q';  'd_u'; 'd_Ch';  'd_Co';};
                obj.FGMstruct.dim =  [ u_dim; handDofs;      c; handDofs;  u_dim;      c;       c;];
                obj.FGMstruct.pointers = [2;4]; % force and displacement pointers for the last level
                obj.FGMstruct.levels = 1;
                obj.FGMstruct.num_equations = 5;

            elseif nargin < 1,
                error('MATLAB:narginchk:notEnoughInputs', 'Not enough input arguments.');
            else
                % copy
                arg1 = varargin{1};
                if ~isa(arg1, 'GraspAnalysisSolver'),
                    error('TRG:GraspAnalysisSolver:NonConsistentValue', 'Wrong input argument: expected a GraspAnalysisSolver object.');
                end
                if numel(arg1) ~= 1,
                    error('TRG:GraspAnalysisSolver:NonConsistentValue', 'Wrong input argument: expected a GraspAnalysisSolver, not an array of GraspAnalysisSolver objects.');
                end
                % HandWithGlove, ObjectWithCover, etc. already copied by super()
                obj.FGMstruct = arg1.FGMstruct;
                obj.q = arg1.q;
                obj.u = arg1.u;
            end
        end

        function obj2 = copy(obj)
            obj2 = GraspAnalysisSolver(obj);
        end
        
        function display(obj)
            disp(' ');
            disp(obj.name);
            k = size(obj.FGMstruct.FGM);
            j = size(obj.FGMstruct.conf);
            disp(['Levels: ',num2str(obj.FGMstruct.levels),', FGM: [',num2str(k(1)), ...
                'x',num2str(k(2)),']',', Configuration: ',num2str(j(1)),' elements']);
            fgmDispConf(obj);
            
        end

        % dependent properties

        function mat = get.FGM(obj)
            mat = obj.FGMstruct.FGM;
        end

    end
    methods (Access = protected)
        structOut = nullSpaceDecomp(obj, set_zero, not_zero);

        % protected methods for managing the FGM config

        function A = fgmStructAddConf(grsp_poe,s,n)
            A = grsp_poe.FGMstruct;
            for j = 1:numel(n),
                a = size(A.conf,1);
                A.conf(a+1) = s(j);
                A.dim(a+1) = n(j);
            end
        end
        
        function [dim,pos,col,flag] = fgmStructFindConf(grsp_poe,s)
            col = 1;
            flag = 0;
            for i = 1 : size(grsp_poe.FGMstruct.conf,1)
                if(strcmp(grsp_poe.FGMstruct.conf{i},s))
                    flag = 1;
                    break
                else col = col + grsp_poe.FGMstruct.dim(i);
                end
            end
            if ~flag,
                error('TRG:GraspAnalysisSolver:UnknownProperty', ...
                        ['Property ', s,' does not exist in the configuration']);
            end
            pos = i;
            dim = grsp_poe.FGMstruct.dim(i);
        end

        function A = fgmStructShiftConf(grsp_poe,s)
            A = grsp_poe.FGMstruct;
            for j = 1:numel(s),
                col = 1;
                for i = 1 : size(A.conf,1)
                    if(strcmp(A.conf{i},s{j}))
                        break
                    end
                    col = col + A.dim(i);
                end

                % shift of the configuration
                n = A.dim(i);
                B = A.FGM(:,col:(col+n)-1);
                A.FGM(:,col:(col+n)-1) = [];
                A.FGM = [A.FGM B];

                % it makes some checks to see how to update the pointers configuration
                if( i == A.pointers(1))    % shifting a generalized force
                    A.pointers(1) = size(A.conf,1);
                    if(i < A.pointers(2))
                        A.pointers(2) = A.pointers(2) -1;
                    end
                else
                    if( i == A.pointers(2))    % shifting a generalized displacement
                        A.pointers(2) = size(A.conf,1);
                        if(i < A.pointers(1))
                         A.pointers(1) = A.pointers(1) -1;
                        end
                    else
                        % not shifting neither a generalized force nor a displacement
                        if(i < A.pointers(1))
                            if(i < A.pointers(2))
                                A.pointers(1) = A.pointers(1) -1;
                                A.pointers(2) = A.pointers(2) -1;
                            else
                                A.pointers(1) = A.pointers(1) -1;
                            end
                        else
                            if(i < A.pointers(2))
                                A.pointers(2) = A.pointers(2) -1;
                            else
                            end
                        end
                    end
                end

                a = A.conf(i);
                A.conf(i) = [];
                A.conf(size(A.conf,1)+1) = a;

                b = A.dim(i);
                A.dim(i) = [];
                A.dim(size(A.dim,1)+1) = b;
            end
        end

        function fgmDispConf(grsp_poe)
            disp('+--------------+-------------+-------------+');
            disp('|   Element    |  Dimension  |   Pointer   |');
            disp('+--------------+-------------+-------------+');

            for i=1: size(grsp_poe.FGMstruct.conf,1)
                pun = '         |';
                if(i == grsp_poe.FGMstruct.pointers(1))
                    pun = 'f_gen    |';
                else
                if(i == grsp_poe.FGMstruct.pointers(2))
                    pun = 'q_gen    |';
                end
                end
                a = size(grsp_poe.FGMstruct.conf{i},2);
                switch(a)
        case 3
            disp(['|     ',grsp_poe.FGMstruct.conf{i},'      |','      ',num2str(grsp_poe.FGMstruct.dim(i)),'      |    ',pun]);
        case 4
            disp(['|     ',grsp_poe.FGMstruct.conf{i},'     |','      ',num2str(grsp_poe.FGMstruct.dim(i)),'      |    ',pun]);
        case 5
            disp(['|     ',grsp_poe.FGMstruct.conf{i},'    |','      ',num2str(grsp_poe.FGMstruct.dim(i)),'      |    ',pun]);
        case 6
            disp(['|     ',grsp_poe.FGMstruct.conf{i},'   |','      ',num2str(grsp_poe.FGMstruct.dim(i)),'      |    ',pun]);
        case 7
            disp(['|     ',grsp_poe.FGMstruct.conf{i},'  |','      ',num2str(grsp_poe.FGMstruct.dim(i)),'      |    ',pun]);
        case 8
            disp(['|   ',grsp_poe.FGMstruct.conf{i},'   |','      ',num2str(grsp_poe.FGMstruct.dim(i)),'      |    ',pun]);
        otherwise
            disp(['|  ',grsp_poe.FGMstruct.conf{i},'   |','      ',num2str(grsp_poe.FGMstruct.dim(i)),'      |    ',pun]);
                end
            end
        disp('+--------------+-------------+-------------+');
        end

        % protected methods for managing transforms of
        % methods' out values from structs to matrices
        % and the opposite
        function [out_m] = methodOutStruct2Mat(grsp_poe, out_s) %#ok<MANU>
            narginchk(2,2);
            % assuming out_s is a struct (errcheck is
            % already performed by struct2cell())
            out_m = [];
            if ~isempty(out_s),
                out_m = cell2mat( struct2cell (out_s) );
            end
        end

        function [out_s] = methodOutMat2Struct(grsp_poe, out_m)
            % no errcheck performed, assuming the method is
            % called the right way
            confNames = grsp_poe.FGMstruct.conf;
            confDims = grsp_poe.FGMstruct.dim;
            out_c = mat2cell(out_m, confDims, size(out_m,2));
            out_s = cell2struct(out_c, confNames);
        end
    end

end
