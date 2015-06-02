% LimbDescription. LimbDescription Class
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

classdef LimbDescription < handle

    properties
        g_st0;   % trasformation matrix from base to end-effector frame
    end
    properties (SetAccess = protected)
        P;       % origins of joint coordinates
        Z;       % joint axes
        n;       % limb's dofs (commodity parameter)
        joints;  % vector of joint types: 0 for rotational, 1 for prismatic joints
    end
    properties (Dependent)
        tool;    % commodity alias for g_st0
        config;  % human-friendly representation of joint types
    end

    methods (Abstract)
        obj2 = copy(obj1);
    end
    methods
        % set/get methods
        function set.g_st0(obj, transform)
            if ~isnumeric(transform),
                error('TRG:setBaseToEEFrame:NonConsistentValue', 'Wrong value, expected a numeric matrix.');
            end
            if ~ishomog(transform),
            	error('TRG:setBaseToEEFrame:NonConsistentValue', 'Wrong value, expected a homogeneous transform');
            end
            obj.g_st0 = transform;
        end
        function set.tool(obj, transform)
            obj.g_st0 = transform;
        end
        function transform = get.tool(obj)
            transform = obj.g_st0;
        end
        function repr = get.config(obj)
            sJointTypes = ['R','P'];
            repr = sJointTypes((obj.joints)+1);
        end
    end

end