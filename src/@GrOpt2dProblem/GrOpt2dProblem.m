% GrOptForceClosure. Data Structure Encapsulating Default Options for 2D Grasps
%   Description, in progress
%
%   Annotations, in progress
%
%   References, in progress
%
%   Matteo Morelli, I.R.C. "E. Piaggio", University of Pisa
%
%   May 2012

% Copyright (C) 2012 Interdepartmental Research Center "E. Piaggio", University of Pisa
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

classdef GrOpt2dProblem < handle

    properties (GetAccess = protected, Constant)
        availablePlanes = 'xy';
        availablePlanesAltern = 'yx';
    end
    properties
        tol;
    end
    properties (SetAccess = protected)
        plane;
    end
    properties (Dependent)
        isSet;
    end
    properties (Access = protected)
        pIsSet;
        poiFramesNormalLocal;
        ignoredForceVelComponents;      % row indices of, e.g., J_c(q)
        ignoredForceVelComponentsBase;  % row indices of, e.g., J_0(q)
        ignoredHtRows;                  % row indices of T (homogeneous transform)
        ignoredHtColumns;               % column indices of T (homogeneous transform)
        ignoredObjComponents;           % row indices of, e.g., u, G (i.e., w), etc.
    end

    methods
        function obj = GrOpt2dProblem(varargin)
            % defaults
            obj.isSet = false;
            obj.tol = 1e-12;
            obj.plane = 'xy';
            obj.poiFramesNormalLocal = [0;0;1];
            obj.ignoredForceVelComponents = [];
            obj.ignoredForceVelComponentsBase = [];
            obj.ignoredHtRows = [];
            obj.ignoredHtColumns = [];
            obj.ignoredObjComponents = [];

            % process user inputs
            if nargin == 1,
                if isa(varargin{1}, 'GrOpt2dProblem'),
                    if length(varargin{1}) ~= 1,
                        error('TRG:GrOpt2dProblem:NonConsistentValue', ...
                                'Wrong input argument: expected a single GrOpt2dProblem object, not an array of objects.');
                    end
                    arg1 = varargin{1};
                    obj.isSet = arg1.isSet;
                    obj.tol = arg1.tol;
                    obj.plane = arg1.plane;
                    obj.poiFramesNormalLocal = arg1.poiFramesNormalLocal;
                    obj.ignoredForceVelComponents = arg1.ignoredForceVelComponents;
                    obj.ignoredForceVelComponentsBase = arg1.ignoredForceVelComponentsBase;
                    obj.ignoredHtRows = arg1.ignoredHtRows;
                    obj.ignoredHtColumns = arg1.ignoredHtColumns;
                    obj.ignoredObjComponents = arg1.ignoredObjComponents;
                    return;
                end
            end
            if nargin > 1,
                % varargin
                i = 1;
                while i < length(varargin),
                    processedProp = varargin{i};
                    switch processedProp,
                        case 'isSet',
                            i = i + 1;
                            if ~isa(varargin{i}, 'logical'),
                                error('TRG:GrOpt2dProblem:NonConsistentValue', ...
                                    'Wrong value specified for ''isSet'', expected a logical.');
                            end
                            if numel(varargin{i}) > 1,
                                error('TRG:GrOpt2dProblem:NonConsistentValue', ...
                                    'Wrong value specified for ''isSet'', expected a single logical, not an array.');
                            end
                            obj.isSet = varargin{i};
                        case 'tol',
                            i = i + 1;
                            tol = varargin{i};
                            if ~isa(tol, 'numeric'),
                                error('TRG:GrOpt2dProblem:NonConsistentValue', ...
                                    'Wrong input argument, expected a tolerance value.');
                            end
                            tol = real(tol(:));
                            if numel(tol) ~= 1,
                                error('TRG:GrOpt2dProblem:NonConsistentValue', ...
                                    'Wrong tolerance value, expected a scalar in place of an array with %d elements.', ...
                                        numel(tol));
                            end
                            obj.tol = tol;
                        case 'plane',
                            i = i + 1;
                            if ~ischar(varargin{i}),
                                error('TRG:GrOpt2dProblem:NonConsistentValue', ...
                                    'Wrong value specified for ''plane'', expected a string.');
                            end
                            [plRows, plCols] = size(varargin{i});
                            if plRows > 1,
                                error('TRG:GrOpt2dProblem:NonConsistentValue', ...
                                    'Wrong value specified for ''plane'': expected a string, not an array of strings.');
                            end
                            if plCols < 1,
                                error('TRG:GrOpt2dProblem:NonConsistentValue', ...
                                    'Wrong value specified for ''plane'': got an empty string.');
                            end
                            availPlaneCmd = 'availablePlanes';
                            atRow_ = 0; %#ok<NASGU>
                            [doesExist, atRow] = ismember(varargin{i}, obj.availablePlanes, 'rows'); %#ok<NASGU>
                            if ~doesExist,
                                availPlaneCmd = 'availablePlanesAltern';
                                [doesExist, atRow_] = ismember(varargin{i}, obj.availablePlanesAltern, 'rows'); %#ok<NASGU>
                            end
                            if ~doesExist,
                                error('TRG:GrOpt2dProblem:NonConsistentValue', ...
                                    'Wrong value specified for ''plane'', plane not supported.');
                            end
                            obj.plane = eval(['obj.', availPlaneCmd, '(atRow+atRow_,:)']);
                        otherwise,
                            error('TRG:GrOpt2dProblem:UnknownProperty', ...
                                'Unknown property passed to GrOpt2dProblem ''%s''.', processedProp);
                    end
                    i = i + 1;
                end
                if obj.isSet,
                    switch obj.plane,
                        case {'xy','yx'},
                            obj.poiFramesNormalLocal = [1;0;0];
                        otherwise,
                            % cannot go here...
                    end
                end
            end
        end

        function obj2 = copy(obj)
            obj2 = GrOpt2dProblem(obj);
        end

        function d = dim(obj)
            d = (~obj.isSet)*3 + 3;
        end

        function d = dimPos(obj)
            d = obj.isSet + 2;
        end

        function res = isEqual(obj1, obj2)
            if ~isa(obj2, 'GrOpt2dProblem'),
                error('TRG:GrOpt2dProblem:NonConsistentValue', ...
                    'Wrong input argument, expected a GrOpt2dProblem object.');
            end
            if numel(obj2) ~= 1,
                error('TRG:GrOpt2dProblem:NonConsistentValue', ...
                    'Wrong input argument: expected a single GrOpt2dProblem object, not an array of objects.');
            end
            res = obj1 == obj2;
        end

        function [modelIsConsistent, idx] = checkConsistencyAndSetIgnoredComponents(obj, Tr)
            modelIsConsistent = true;
            idx = [];
            switch obj.plane,
                case {'xy', 'yx'},
                    v = [0;0;1];
                    pfn_idx = find(obj.poiFramesNormalLocal, 1);
                    ignoredForceVelCompsBase = [3,4,5];
                    ignoredHtRows = 3; %#ok<*CPROP>
                    ignoredHtCols = 3;
                    ignoredObjComps = [3,4,5];
                otherwise,
                    error('TRG:GrOpt2dProblem:NonConsistentValue', ...
                            'Wrong value specified for ''plane'', plane not supported.');
            end
            % now check and assign values to .ignoredForceVelComponents, etc.
            dimTr = numel(Tr);
            obj.ignoredForceVelComponents = zeros(3*dimTr,1);
            obj.ignoredForceVelComponentsBase = zeros(3*dimTr,1);
            obj.ignoredHtRows = ignoredHtRows;
            obj.ignoredHtColumns = ignoredHtCols;
            obj.ignoredObjComponents = ignoredObjComps;
            complementary_pfn_idx = 1:3;
            complementary_pfn_idx(pfn_idx) = [];
            tol = obj.tol;
            for j = 1:dimTr,
                ignore_this_idx = 1:6;
                % check is poiFrameNormal (in Local coords) is orthogonal to v
                if abs(Tr{j}(1:3,pfn_idx)'*v) > tol,
                    % error
                    modelIsConsistent = false;
                    idx = [idx, j]; %#ok<AGROW>
                else
                    % now check for the other two components
                    if abs(Tr{j}(1:3,complementary_pfn_idx(1))'*v) < tol,
                        % x is the other orthogonal component
                        ignore_this_idx([pfn_idx, complementary_pfn_idx(1), 3+complementary_pfn_idx(2)]) = [];
                    elseif abs(Tr{j}(1:3,complementary_pfn_idx(2))'*v) < tol,
                        % y is the other orthogonal component
                        ignore_this_idx([pfn_idx, complementary_pfn_idx(2), 3+complementary_pfn_idx(1)]) = [];
                    else
                        % error
                        modelIsConsistent = false;
                        idx = [idx, j]; %#ok<AGROW>
                    end
                end
                obj.ignoredForceVelComponents(3*(j-1)+(1:3)) = 6*(j-1)+ignore_this_idx;
                obj.ignoredForceVelComponentsBase(3*(j-1)+(1:3)) = 6*(j-1)+ignoredForceVelCompsBase;
            end

        end

        function vec3d = restoreVectorTo3d(obj, vec2d)
            if ~isa(vec2d, 'numeric'),
                error('TRG:GrOpt2dProblem:NonConsistentValue',...
                        'Wrong input argument, expected a 2D vector.');
            end
            vec2d = vec2d(:);
            if numel(vec2d) ~= 2,
                error('TRG:GrOpt2dProblem:NonConsistentValue', ...
                    'Wrong object config, expected 2 elements in place of %d.', ...
                        numel(vec2d));
            end
            switch obj.plane,
                case {'xy','yx'},
                    vec3d = [vec2d(1:2);...        % ..
                                   0];             % position
                otherwise,
                    error('TRG:GrOpt2dProblem:NonConsistentValue', ...
                            'Wrong value specified for ''plane'', plane not supported.');
            end
        end

        function conf3d = restoreConfTo3d(obj, conf2d)
            if ~isa(conf2d, 'numeric'),
                error('TRG:GrOpt2dProblem:NonConsistentValue',...
                        'Wrong input argument, expected a 2D object configuration vector.');
            end
            conf2d = conf2d(:);
            if numel(conf2d) ~= 3,
                error('TRG:GrOpt2dProblem:NonConsistentValue', ...
                    'Wrong object config, expected 3 elements in place of %d.', ...
                        numel(conf2d));
            end
            switch obj.plane,
                case {'xy','yx'},
                    conf3d = [conf2d(1:2);...        % ..
                                   0;                % position
                       tr2eul(trotz(conf2d(3)))'];   % orientation
                otherwise,
                    error('TRG:GrOpt2dProblem:NonConsistentValue', ...
                            'Wrong value specified for ''plane'', plane not supported.');
            end
        end

        function M_2d = reduceDiffMotionsMapTo2d(obj, M)
            M_2d = M;
            M_2d(obj.ignoredForceVelComponents,:) = [];
        end

        function B_2d = reduceDiffMotionsMapBaseTo2d(obj, B)
            B_2d = B;
            B_2d(obj.ignoredForceVelComponentsBase,:) = [];
        end

        function T_2d = reduceHomogTransformTo2d(obj, T)
            T_2d = T;
            T_2d(obj.ignoredHtRows, :) = [];
            T_2d(:, obj.ignoredHtColumns) = [];
        end

        function U_2d = reduceObjectComponentsTo2d(obj, U)
            U_2d = U;
            U_2d(obj.ignoredObjComponents,:) = [];
        end

        function n = frameNormalLocal(obj)
            n = obj.poiFramesNormalLocal;
            if obj.isSet,
                n(obj.ignoredHtRows, :) = [];
            end
        end

        function n = frameNormal(obj, T)
            if obj.isSet,
                T = obj.reduceHomogTransformTo2d(T);
            end
            n = T(1:end-1,find(obj.poiFramesNormalLocal, 1));
        end

        %   set/get methods  %

        function res = get.isSet(obj)
            res = obj.pIsSet;
        end

        function set.isSet(obj, val)
            if ~isa(val, 'logical'),
                error('TRG:GrOpt2dProblem:NonConsistentValue', ...
                    'Wrong value specified for ''isSet'', expected a logical (true/false).');
            end
            if numel(val) > 1,
                error('TRG:GrOpt2dProblem:NonConsistentValue', ...
                    'Wrong value specified for ''isSet'', expected a single logical, not an array.');
            end
            obj.pIsSet = val;
            if val,
                switch obj.plane,
                    case {'xy','yx'},
                        obj.poiFramesNormalLocal = [1;0;0];
                    otherwise,
                        % cannot go here...
                end
            end
        end

        function set.tol(obj, tol)
            if ~isa(tol, 'numeric'),
                error('TRG:GrOpt2dProblem:NonConsistentValue', ...
                    'Wrong input argument, expected a tolerance value.');
            end
            tol = real(tol(:));
            if numel(tol) ~= 1,
                error('TRG:GrOpt2dProblem:NonConsistentValue', ...
                    'Wrong tolerance value, expected a scalar in place of an array with %d elements.', ...
                    numel(tol));
            end
            obj.tol = tol;
        end

%{
        function set.plane(obj, val)
            if ~ischar(val),
                error('TRG:GrOpt2dProblem:NonConsistentValue', ...
                    'Wrong value specified for ''plane'', expected a string.');
            end
            if size(val, 1) > 1,
                error('TRG:GrOpt2dProblem:NonConsistentValue', ...
                    'Wrong value specified for ''plane'': expected a string, not an array of strings.');
            end
            availPlaneCmd = 'availablePlanes';
            atRow_ = 0; %#ok<NASGU>
            [doesExist, atRow] = ismember(val, obj.availablePlanes, 'rows'); %#ok<NASGU>
            if ~doesExist,
                availPlaneCmd = 'availablePlanesAltern';
                [doesExist, atRow_] = ismember(val, obj.availablePlanesAltern, 'rows'); %#ok<NASGU>
            end
            if ~doesExist,
                error('TRG:GrOpt2dProblem:NonConsistentValue', ...
                    'Wrong value specified for ''plane'', plane not supported.');
            end
            obj.plane = eval(['obj.', availPlaneCmd, '(atRow+atRow_,:)']);
        end
%}

        %  operators  %

        function res = eq(a, b)
            res = (a.isSet == b.isSet) && ...
                    (strcmp(a.plane,b.plane) || ...
                        strcmp(a.plane,b.plane(end:-1:1)));
        end

        function res = ne(a, b)
            res = ~eq(a,b);
        end

    end
end