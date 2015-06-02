% ClosedChainsSolver. A Solver for Rigid Closed-Chain Mechanisms
%   Description, in progress
%
%   Annotations, in progress
%
%   References, in progress
%
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

classdef ClosedChainsSolver < GraspSolver

    properties
        activeJoints;
    end

    methods (Access = protected)
        grM = subspaceDecomp(obj, q, u, sdCbk);
    end
    methods
        function obj = ClosedChainsSolver(varargin)
            % collect properties
            [oplCCS, oplOther] = srv_filter_options_from_list({'activeJoints'}, 1, varargin{:});

            % super()
            %
            % set HandWithGlove, ObjectWithCover, contact struct (required!)
            % and name, comment, GrOpt2dProblem (if any)
            obj@GraspSolver(oplOther{:});

            if nargin > 1,
                % defaults
                handDofs = obj.handWithGlove.hand.dofs;
                actJoints = (1:handDofs);  % default: fully actuated hand
                % set activeJoints (if any)
                if length(oplCCS) > 1,
                    actJoints = (unique(real(oplCCS{2}(:))))';
                    if actJoints(end) > handDofs,
                        error('TRG:ClosedChainsSolver:NonConsistentValue', ...
                            'Wrong value specified for active joints, at least a value is greater than %d.', ...
                                handDofs);
                    end
                end
                obj.activeJoints = actJoints;
            elseif nargin < 1,
                error('MATLAB:narginchk:notEnoughInputs', 'Not enough input arguments.');
            else
                % copy
                arg1 = varargin{1};
                if ~isa(arg1, 'ClosedChainsSolver'),
                    error('TRG:ClosedChainsSolver:NonConsistentValue', 'Wrong input argument: expected a ClosedChainsSolver object.');
                end
                if numel(arg1) ~= 1,
                    error('TRG:ClosedChainsSolver:NonConsistentValue', 'Wrong input argument: expected a ClosedChainsSolver, not an array of ClosedChainsSolver objects.');
                end
                % HandWithGlove, ObjectWithCover, etc. already copied by super()
                obj.activeJoints = arg1.activeJoints;
            end
        end

        function obj2 = copy(obj)
            obj2 = ClosedChainsSolver(obj);
        end

        function set.activeJoints(obj, val)
            handDofs = obj.handWithGlove.hand.dofs;
            actJoints = (unique(real(val(:))))';
            if actJoints(end) > handDofs,
                error('TRG:ClosedChainsSolver:NonConsistentValue', ...
                    'Wrong value specified for active joints, at least a value is greater than %d.', ...
                        handDofs);
            end
            obj.activeJoints = actJoints;
        end
    end

end