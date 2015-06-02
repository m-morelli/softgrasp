% LimbDescriptionScrew. LimbDescriptionScrew Class
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

classdef LimbDescriptionScrew < LimbDescription

    methods

        function obj = LimbDescriptionScrew(Z, P, g_st0, varargin)
            %
            % SYNOPSYS
            % l2 = LimbDescriptionScrew(l1)
            % l1 = LimbDescriptionScrew(Z, P, g_st0)
            % l1 = LimbDescriptionScrew(Z, P, g_st0, 'PropertyName', PropertyValue, ...)
            %

            % default props
            obj.n = 0;
            obj.P = [];
            obj.Z = [];
            obj.g_st0= eye(4);
            obj.joints = [];

            % process user-providen options
            if nargin < 1,
                error('MATLAB:narginchk:notEnoughInputs', 'Not enough input arguments.');
            end
            if isa(Z, 'LimbDescriptionScrew'),
                if nargin > 1,
                    error('MATLAB:narginchk:tooManyInputs', 'Too many input arguments.');
                end
                if length(Z) ~= 1,
                    error('TRG:LimbDescriptionScrew:NonConsistentValue', 'Wrong input argument: expected a LimbDescriptionScrew, not an array of LimbDescriptionScrew objects.');
                end
                % copy
                obj.n = Z.n;
                obj.P = Z.P;
                obj.Z = Z.Z;
                obj.g_st0 = Z.g_st0;
                obj.joints = Z.joints;
            elseif isnumeric(Z),
                %narginchk(3, 4);
                if nargin < 3 || nargin == 4,
                    error('MATLAB:narginchk:notEnoughInputs', 'Not enough input arguments.');
                end
                if nargin > 5,
                    error('MATLAB:narginchk:tooManyInputs', 'Too many input arguments.');
                end
                if size(Z,1) ~= 3,
                    error('TRG:LimbDescriptionScrew:NonConsistentValue', 'Wrong input argument: the matrix of joint axes must be 3xn.');
                end
                dofs = size(Z,2);
                joints = zeros(1,dofs); %#ok<*PROP>
                if ~isnumeric(P),
                    error('TRG:LimbDescriptionScrew:NonConsistentValue', 'Wrong input argument: the matrix of origins of joint coordinates must be numeric.');
                end
                if any(size(P) ~= [3,dofs])
                    error('TRG:LimbDescriptionScrew:NonConsistentValue', 'Wrong input argument: the matrix of origins of joint coordinates must be 3x%d.', dofs);
                end
                if ~isnumeric(g_st0),
                    error('TRG:LimbDescriptionScrew:NonConsistentValue', 'Wrong value specified for the base to end-effector transform matrix, expected a numeric matrix.');
                end
                if any(size(g_st0) ~= [4,4]),
                    error('TRG:LimbDescriptionScrew:NonConsistentValue', 'Wrong value specified for the base to end-effector transform matrix, expected a numeric 4x4 matrix.');
                end
                if nargin > 3,
                    propName = varargin{1};
                    if ~ischar(propName),
                        error('TRG:LimbDescriptionScrew:NonConsistentValue', 'Wrong value specified for ''%s'', expected a string.', propName);
                    end
                    if size(propName, 1) > 1,
                        error('TRG:LimbDescriptionScrew:NonConsistentValue', 'Wrong value specified for ''%s'': expected a string, not an array of strings.', propName);
                    end
                    if ~strcmp(propName, 'joints'),
                        error('TRG:LimbDescriptionScrew:UnknownProperty', 'Unknown property ''%s''.', propName);
                    end
                    if ~isnumeric(varargin{2}),
                        error('TRG:LimbDescriptionScrew:NonConsistentValue', 'Wrong value specified for ''joints'', expected a numeric vector.');
                    end
                    joints = (varargin{2}(:))';
                    if size(joints, 2) ~= dofs,
                        error('TRG:LimbDescriptionScrew:NonConsistentValue', 'Wrong value specified for ''joints'', expected a numeric vector with %d elements.', dofs);
                    end
                    if ~(min(joints) >= 0 && max(joints) <= 1),
                        error('TRG:LimbDescriptionScrew:NonConsistentValue', 'Wrong value specified for ''joints'', expected a numeric vector with 0s and 1s');
                    end
                end
                obj.Z = Z;
                obj.P = P;
                obj.g_st0 = g_st0;
                obj.joints = joints;
                obj.n = dofs;
            else
                error('TRG:LimbDescriptionScrew:UnknownInput', 'Wrong input argument, expected a LimbDescriptionScrew or numeric matrices.');
            end
        end

        function obj2 = copy(obj1)
            %LimbDescriptionScrew.copy Clone a LimbDescriptionScrew object
            %
            % l2 = l1.copy() is a deepcopy of the object l1.
            obj2 = LimbDescriptionScrew(obj1);
        end

    end

end