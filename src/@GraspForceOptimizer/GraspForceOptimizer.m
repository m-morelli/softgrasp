% GraspForceOptimizer. A Class Providing Facilities for Grasp-Force Optimization (GFO)
%   Description, in progress
%
%   Annotations, in progress
%
%   References, in progress
%
%   Matteo Morelli, I.R.C. "E. Piaggio", University of Pisa
%
%   July 2013
%   Modified on October 2013:
%       - alignment of legacy code according to the new class organization

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

classdef GraspForceOptimizer < handle

    properties
        name;
        comment;
        excludeContacts;
        extCtForceTransmission;
        controllableGraspForces;
        minF;
        maxF;
        de;
        wf;
        wm;
        wM;
        y0;
        stepSolver;
        tol;
        max_stepn;
    end
    properties (SetAccess = protected)
        contactStructure; % - cannot change the cs (SetAccess=protected);
                          % build another instance if you want a different cs
                          % - cannot set directly matches and ctModels via
                          %     gfo.contactStructure.matches = [...], or
                          %     gfo.contactStructure.contactModels(i) = hf
                          % (because matches and contactModels are
                          % SetAccess=protected in ContactStructure)
                          % - setting the properties of a contact is allowed, eg. via
                          %     gfo.contactStructure.contactModels(i).prop = ...
                          % however, the only properties that can be set are:
                          %     # name    (un-useful for simulations)
                          %     # comment (un-useful for simulations)
                          %     # frictionCoefficients
                          % enabling setting of frictionCoefficients is
                          % fine, so we can leave SetAccess=protected for
                          % contactStructure without resorting to
                          % complicate managements such as DeepCopies etc.

    end
    properties (Access = protected)
        pReducedProb;
        wExt; % internal use only (used by optimize())
    end
    properties (Dependent)
        reducedProb;
    end

    methods (Access = protected)
        [v,dv,hv,s] = llcs_vdvhv(go, Fe, yo);
        [la, jj]    = llcs_vdvhvss_gss(go, y_k, d_k);
        [lx_, jj]   = llcs_vdvhvss_bis(go, y_k, d_k);
    end
    methods
        function obj = GraspForceOptimizer(arg1, mWf, Rf, varargin)
            % defaults:
            % (obj.contactStructure, obj.extCtForceTransmission,
            %  obj.controllableGraspForces are required)
            obj.name = 'noname';
            obj.comment = '';
            obj.pReducedProb = GrOpt2dProblem;
            obj.excludeContacts = [];
            obj.minF = 1e-2;
            obj.maxF = 1e2;
            obj.de = 1e-5;
            obj.wf = 1;
            obj.wm = 1;
            obj.wM = 1;
            obj.y0 = [];
            obj.stepSolver = 'gss';
            obj.tol = 1e-10;
            obj.max_stepn = 1000;
            % process required user-providen options
            if nargin < 1,
                error('MATLAB:narginchk:notEnoughInputs', 'Not enough input arguments.');
            end
            if isa(arg1, 'GraspForceOptimizer'),
                % Copy
                if numel(arg1) ~= 1,
                    error('TRG:GraspForceOptimizer:NonConsistentValue',...
                        'Wrong input argument: expected a GraspForceOptimizer, not an array of GraspForceOptimizers.');
                end
                obj.name = arg1.name;
                obj.comment = arg1.comment;
                obj.pReducedProb = arg1.pReducedProb.copy;
                obj.contactStructure = arg1.contactStructure.copy;
                obj.extCtForceTransmission = arg1.extCtForceTransmission;
                obj.controllableGraspForces = arg1.controllableGraspForces;
                obj.excludeContacts = arg1.excludeContacts;
                obj.minF = arg1.minF;
                obj.maxF = arg1.maxF;
                obj.de = arg1.de;
                obj.wf = arg1.wf;
                obj.wm = arg1.wm;
                obj.wM = arg1.wM;
                obj.y0 = arg1.y0;
                obj.stepSolver = arg1.stepSolver;
                obj.tol = arg1.tol;
                obj.max_stepn = arg1.max_stepn;
            else
                % check for ContactStructure, Grk, E
                if nargin < 3,
                    error('MATLAB:narginchk:notEnoughInputs', 'Not enough input arguments.');
                end
                if ~isa(arg1, 'ContactStructure'),
                    error('TRG:GraspForceOptimizer:NonConsistentValue',...
                            'Wrong input argument, expected a ContactStructure object.');
                end
                if numel(arg1) ~= 1,
                    error('TRG:GraspForceOptimizer:NonConsistentValue',...
                            'Wrong input argument: expected a ContactStructure, not an array of ContactStructures.');
                end
                if isempty(cell2mat({arg1.contactModels.contactFrameObjectSide})),
                    error('TRG:GraspForceOptimizer:NonConsistentValue',...
                            'Contact frames are not initialized, did you extract the ContactStructure instance from a GraspAnalysisSolver instance?');
                end
                if ~isa(mWf, 'numeric'),
                    error('TRG:GraspForceOptimizer:NonConsistentValue', ...
                        'Wrong input argument, the value of ''extCtForceTransmission'' must be a numeric matrix.');
                end
                if ~isa(Rf, 'numeric'),
                    error('TRG:GraspForceOptimizer:NonConsistentValue', ...
                        'Wrong input argument, the value of ''controllableGraspForces'' must be a numeric matrix.');
                end
                obj.contactStructure = arg1.copy;
                obj.extCtForceTransmission = real(mWf);
                obj.controllableGraspForces = real(Rf);
                %
                % check for optional user-providen options
                if nargin > 3,
                    i = 1;
                    lOptVar = length(varargin);
                    while i < lOptVar,
                        processedProp = varargin{i};
                        switch processedProp,
                            case {'name', 'comment'},
                                currentProp = varargin{i};
                                i = i + 1;
                                if ~ischar(varargin{i}),
                                    error('TRG:GraspForceOptimizer:NonConsistentValue', ...
                                        'Wrong value specified for ''%s'', expected a string.', ...
                                            currentProp);
                                end
                                if size(varargin{i}, 1) > 1,
                                    error('TRG:GraspForceOptimizer:NonConsistentValue', ...
                                        'Wrong value specified for ''%s'': expected a string, not an array of strings.', ...
                                            currentProp);
                                end
                                obj.(currentProp) = varargin{i};
                            case 'reducedProb',
                                i = i + 1;
                                if ~isa(varargin{i}, 'GrOpt2dProblem'),
                                    error('TRG:GraspForceOptimizer:NonConsistentValue', ...
                                        'Wrong value specified for ''reducedProb'', expected a GrOpt2dProblem object.');
                                end
                                obj.pReducedProb = varargin{i}.copy;
                            case 'excludeContacts',
                                i = i + 1;
                                if ~isa(varargin{i}, 'numeric'),
                                    error('TRG:GraspForceOptimizer:NonConsistentValue', ...
                                        'Wrong input argument, expected a set of indices.');
                                end
                                if ~any(size(varargin{i})< 2),
                                    error('TRG:GraspForceOptimizer:NonConsistentValue', ...
                                        'Wrong input argument; expected an array, not a matrix of indices.');
                                end
                                obj.excludeContacts = unique(varargin{i});
                            case {'minF', 'maxF', 'de', 'wf', 'wm', 'wM', 'tol', 'max_stepn'},
                                currentProp = varargin{i};
                                i = i + 1;
                                if ~isa(varargin{i}, 'numeric'),
                                    error('TRG:GraspForceOptimizer:NonConsistentValue', ...
                                        'Wrong value specified for ''%s'', expected a numeric value.',...
                                            currentProp);
                                end
                                if numel(varargin{i}) ~= 1,
                                    error('TRG:GraspForceOptimizer:NonConsistentValue', ...
                                        'Wrong value specified for ''%s'', expected a numeric value.',...
                                            currentProp);
                                end
                                obj.(currentProp) = varargin{i};
                            case 'y0',
                                if ~isa(varargin{i}, 'numeric'),
                                    error('TRG:GraspForceOptimizer:NonConsistentValue', ...
                                        'Wrong input argument, expected a numeric vector.');
                                end
                                if ~any(size(varargin{i})< 2),
                                    error('TRG:GraspForceOptimizer:NonConsistentValue', ...
                                        'Wrong input argument; expected an array, not a matrix.');
                                end
                                obj.y0 = varargin{i}(:);
                            case 'stepSolver',
                                i = i + 1;
                                if ~ischar(varargin{i}),
                                    error('TRG:GraspForceOptimizer:NonConsistentValue', ...
                                        'Wrong value specified for ''stepSolver'', expected a string.');
                                end
                                if size(varargin{i}, 1) > 1,
                                    error('TRG:GraspForceOptimizer:NonConsistentValue', ...
                                        'Wrong value specified for ''stepSolver'': expected a string, not an array of strings.');
                                end
                                obj.stepSolver = varargin{i};
                            otherwise,
                                i = i + 1;
                                % ignore properties that may be specific of
                                % subclasses (if any)
                        end
                        i = i + 1;
                    end
                end
            end
        end

        function obj2 = copy(obj)
            obj2 = GraspForceOptimizer(obj);
        end

        function rp = getReducedProbDeepCopy(obj)
            rp = obj.pReducedProb.copy;
        end

        % set methods

        function set.name(obj, aString)
            if ~ischar(aString),
                error('TRG:GraspForceOptimizer:NonConsistentValue', ...
                	'Wrong value specified for ''name'', expected a string.');
            end
            if size(aString, 1) > 1,
                error('TRG:GraspForceOptimizer:NonConsistentValue', ...
                	'Wrong value specified for ''name'': expected a string, not an array of strings.');
            end
            obj.name = aString;
        end

        function set.comment(obj, aString)
            if ~ischar(aString),
                error('TRG:GraspForceOptimizer:NonConsistentValue', ...
                	'Wrong value specified for ''comment'', expected a string.');
            end
            if size(aString, 1) > 1,
                error('TRG:GraspForceOptimizer:NonConsistentValue', ...
                	'Wrong value specified for ''comment'': expected a string, not an array of strings.');
            end
            obj.comment = aString;
        end

        function set.excludeContacts(obj, val)
            if ~isa(val, 'numeric'),
                error('TRG:GraspForceOptimizer:NonConsistentValue', ...
                    'Wrong input argument, expected a set of indices.');
            end
            if ~any(size(val) < 2),
                error('TRG:GraspForceOptimizer:NonConsistentValue', ...
                    'Wrong input argument; expected an array, not a matrix of indices.');
            end
            obj.excludeContacts = unique(val);
        end

        function set.extCtForceTransmission(obj, val)
            if ~isa(val, 'numeric'),
                error('TRG:GraspForceOptimizer:NonConsistentValue', ...
                    'Wrong input argument, expected a numeric matrix.');
            end
            obj.extCtForceTransmission = val;
        end

        function set.controllableGraspForces(obj, val)
            if ~isa(val, 'numeric'),
                error('TRG:GraspForceOptimizer:NonConsistentValue', ...
                    'Wrong input argument, expected a numeric matrix.');
            end
            obj.controllableGraspForces = val;
        end

        function set.minF(obj, val)
            if ~isa(val, 'numeric'),
                error('TRG:GraspForceOptimizer:NonConsistentValue', ...
                    'Wrong input argument, expected a numeric value.');
            end
            if numel(val) ~= 1,
                error('TRG:GraspForceOptimizer:NonConsistentValue', ...
                    'Wrong input argument, expected a numeric value.');
            end
            obj.minF = val;
        end

        function set.maxF(obj, val)
            if ~isa(val, 'numeric'),
                error('TRG:GraspForceOptimizer:NonConsistentValue', ...
                    'Wrong input argument, expected a numeric value.');
            end
            if numel(val) ~= 1,
                error('TRG:GraspForceOptimizer:NonConsistentValue', ...
                    'Wrong input argument, expected a numeric value.');
            end
            obj.maxF = val;
        end

        function set.de(obj, val)
            if ~isa(val, 'numeric'),
                error('TRG:GraspForceOptimizer:NonConsistentValue', ...
                    'Wrong input argument, expected a numeric value.');
            end
            if numel(val) ~= 1,
                error('TRG:GraspForceOptimizer:NonConsistentValue', ...
                    'Wrong input argument, expected a numeric value.');
            end
            obj.de = val;
        end

        function set.wf(obj, val)
            if ~isa(val, 'numeric'),
                error('TRG:GraspForceOptimizer:NonConsistentValue', ...
                    'Wrong input argument, expected a numeric value.');
            end
            if numel(val) ~= 1,
                error('TRG:GraspForceOptimizer:NonConsistentValue', ...
                    'Wrong input argument, expected a numeric value.');
            end
            obj.wf = val;
        end

        function set.wm(obj, val)
            if ~isa(val, 'numeric'),
                error('TRG:GraspForceOptimizer:NonConsistentValue', ...
                    'Wrong input argument, expected a numeric value.');
            end
            if numel(val) ~= 1,
                error('TRG:GraspForceOptimizer:NonConsistentValue', ...
                    'Wrong input argument, expected a numeric value.');
            end
            obj.wm = val;
        end

        function set.wM(obj, val)
            if ~isa(val, 'numeric'),
                error('TRG:GraspForceOptimizer:NonConsistentValue', ...
                    'Wrong input argument, expected a numeric value.');
            end
            if numel(val) ~= 1,
                error('TRG:GraspForceOptimizer:NonConsistentValue', ...
                    'Wrong input argument, expected a numeric value.');
            end
            obj.wM = val;
        end

        function set.y0(obj, val)
            if ~isa(val, 'numeric'),
                error('TRG:GraspForceOptimizer:NonConsistentValue', ...
                    'Wrong input argument, expected a numeric value.');
            end
            if ~any(size(val)< 2),
                error('TRG:GraspForceOptimizer:NonConsistentValue', ...
                    'Wrong input argument; expected an array, not a matrix.');
            end
            obj.y0 = val(:);
        end

        function set.stepSolver(obj, val)
            if ~ischar(val),
                error('TRG:GraspForceOptimizer:NonConsistentValue', ...
                    'Wrong value specified for ''stepSolver'', expected a string.');
            end
            if size(val, 1) > 1,
                error('TRG:GraspForceOptimizer:NonConsistentValue', ...
                    'Wrong value specified for ''stepSolver'': expected a string, not an array of strings.');
            end
            switch val,
                case 'gss',
                    obj.stepSolver = @llcs_vdvhvss_gss;
                case 'bis',
                    obj.stepSolver = @llcs_vdvhvss_bis;
                otherwise,
                    error('TRG:GraspForceOptimizer:NonConsistentValue', ...
                        'Wrong value specified for ''stepSolver'': unknown solver.');
            end
        end

        function set.tol(obj, val)
            if ~isa(val, 'numeric'),
                error('TRG:GraspForceOptimizer:NonConsistentValue', ...
                    'Wrong input argument, expected a numeric value.');
            end
            if numel(val) ~= 1,
                error('TRG:GraspForceOptimizer:NonConsistentValue', ...
                    'Wrong input argument, expected a numeric value.');
            end
            obj.tol = val;
        end

        function set.max_stepn(obj, val)
            if ~isa(val, 'numeric'),
                error('TRG:GraspForceOptimizer:NonConsistentValue', ...
                    'Wrong input argument, expected a numeric value.');
            end
            if numel(val) ~= 1,
                error('TRG:GraspForceOptimizer:NonConsistentValue', ...
                    'Wrong input argument, expected a numeric value.');
            end
            obj.max_stepn = val;
        end

        %  dependent  props

        function rp = get.reducedProb(obj) %#ok<STOUT,MANU>
            error('TRG:GraspForceOptimizer:NonPermittedOperation', ...
                'Use ''getReducedProbDeepCopy()'' to get a copy of the GrOpt2dProblem object, or use ''isProblem2d()'' to know its current settings about 2D.');
        end

        function set.reducedProb(obj, rp)
            if ~isa(rp, 'GrOpt2dProblem'),
                error('TRG:GraspForceOptimizer:NonConsistentValue', ...
                        'Wrong value specified for ''reducedProb'', expected a GrOpt2dProblem object.');
            end
            obj.pReducedProb = rp.copy;
            % check consistency if 2D
            if obj.isProblem2d,
                cm = obj.contactStructure.contactModels;
                if any(cellfun('isclass', num2cell(cm), 'SoftFingerContact')),
                    error('TRG:GraspForceOptimizer:NonConsistentValue', ...
                        'Impossible to reduce the problem to 2d, at least one contact is a SoftFingerContact.');
                end
                [modelIsConsistent, idx] = obj.pReducedProb.checkConsistencyAndSetIgnoredComponents({cm.contactFrameObjectSide});
                if ~modelIsConsistent,
                    error('TRG:GraspForceOptimizer:NonConsistentValue', ...
                        'Impossible to reduce the problem to 2d, contact frames %d do not lie onto plane %s.',...
                        mat2str(idx), obj.pReducedProb.plane);
                end
            end
        end
    end

end