% GraspSolver. An Abstract Class for Modeling Grasps and Computing
% Fundamental Matrices for Grasp Analysis
%   Description, in progress
%
%   Annotations, in progress
%
%   References, in progress
%
%   Matteo Morelli, I.R.C. "E. Piaggio", University of Pisa
%   Nicola Greco, I.R.C. "E. Piaggio", University of Pisa
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
% but WITHOUT ANY WARRANTY; without even the implied warranty ofhttps://auth7.unipi.it/auth/perfigo_weblogin.jsp?cm=ws32vklm&uri=http%3A%2F%2F
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
%
% You should have received a copy of the GNU General Public License
% along with THE Robotic Grasping Toolbox for use with MATLAB(R).  If not, see <http://www.gnu.org/licenses/>.

classdef (Abstract) GraspSolver < handle

    properties
        name;
        comment;
    end
    properties (Dependent)
        reducedProb;
        PlotOpt;
    end
    properties (Access = protected)
        pReducedProb;
        pPlotOpt;
    end
    properties (Access = protected, Constant)
        virtualChain = Limb([eye(3),eye(3)], ...                     % Z
                                zeros(3,6), ...                      % P
                                    transl(0,0,0), ...               % g_st0
                                        'joints', [1,1,1,0,0,0], ... % revolute(0)/prismatic(1)
                                            'name', 'virtualChain');
    end
    properties (SetAccess = protected)
        handWithGlove;
        objectWithCover;
        contactStructure;
    end

    methods

        function obj = GraspSolver(varargin)

            % defaults
            obj.name = 'noname';
            obj.comment = '';
            obj.pReducedProb = GrOpt2dProblem;
            obj.pPlotOpt = GrOptPlot;

            % process required user-providen options
            if nargin < 1,
                error('MATLAB:narginchk:notEnoughInputs', 'Not enough input arguments.');
            end
            arg1 = varargin{1};
            if isa(arg1, 'GraspSolver'),
                % Copy
                if numel(arg1) ~= 1,
                    error('TRG:GraspSolver:NonConsistentValue',...
                        'Wrong input argument: expected a GraspSolver, not an array of GraspSolvers.');
                end
                obj.name = arg1.name;
                obj.comment = arg1.comment;
                obj.pReducedProb = arg1.pReducedProb.copy;
                obj.handWithGlove = arg1.handWithGlove.copy;
                obj.objectWithCover = arg1.objectWithCover.copy;
                obj.contactStructure = arg1.contactStructure.copy;
                obj.pPlotOpt = arg1.PlotOpt.copy;
            else
                % check HandWithGlove, ObjectWithCover and CtStruct
                if ~isa(arg1, 'HandWithGlove'),
                    error('TRG:GraspSolver:NonConsistentValue',...
                            'Wrong input argument, expected a HandWithGlove object.');
                end
                if numel(arg1) ~= 1,
                    error('TRG:GraspSolver:NonConsistentValue',...
                            'Wrong input argument: expected a HandWithGlove, not an array of HandWithGlove objects.');
                end
                if nargin < 3,
                    error('MATLAB:narginchk:notEnoughInputs',...
                            'Not enough input arguments.');
                end
                arg2 = varargin{2};
                arg3 = varargin{3};
                if ~isa(arg2, 'ObjectWithCover'),
                    error('TRG:GraspSolver:NonConsistentValue',...
                            'Wrong input argument, expected a ObjectWithCover object.');
                end
                if numel(arg2) ~= 1,
                    error('TRG:GraspSolver:NonConsistentValue',...
                            'Wrong input argument: expected a ObjectWithCover, not an array of ObjectWithCover objects.');
                end
                if ~isa(arg3, 'ContactStructure'),
                    error('TRG:GraspSolver:NonConsistentValue',...
                            'Wrong input argument, expected a ContactStructure object.');
                end
                if numel(arg3) ~= 1,
                    error('TRG:GraspSolver:NonConsistentValue',...
                            'Wrong input argument: expected a ContactStructure, not an array of ContactStructures.');
                end
                % are some GrOpt2dProblem settings already available?
                if ~arg1.getReducedProbDeepCopy.isEqual(arg2.getReducedProbDeepCopy),
                    warning('TRG:GraspSolver:NonConsistentValue',...
                        'Detected different GrOpt2dProblem settings for HandWithGlove and ObjectWithCover, leaving the default one in GraspSolver');
                else
                    obj.pReducedProb = arg2.getReducedProbDeepCopy; % ObjectWithCover (is this actually needed?)
                end
                % 
                % ok, now prepare HandWithGlove, ObjectWithCover, ContactStruct deep copies for data encaps.
                anotherGlove = HandWithGlove(arg1);
                anotherCover = ObjectWithCover(arg2);
                anotherCtStr = ContactStructure(arg3);
                for i = 1:anotherCtStr.nContactModels,
                    anotherCtStr.contactModels(i).contactFrameObjectSide = ...
                        anotherCover.POIStructure.pois(anotherCtStr.matches(i,2)).relativeTransform;
                end
                % TODO: check CtStr for
                %   - glovePOIs in CtStr.matches(:,1)
                %   - coverPOIs in CtStr.matches(:,2)
                %   - all *POIs used ?
                %
                % and assign them to 'this' object (PlotOpts are handled
                % below)
                obj.handWithGlove = anotherGlove;
                obj.objectWithCover = anotherCover;
                obj.contactStructure = anotherCtStr;
                %
                % check for optional user-providen options
                if nargin > 3,
                    i = 4;
                    while i < nargin,
                        processedProp = varargin{i};
                        switch processedProp,
                            case {'name', 'comment'},
                                currentProp = varargin{i};
                                i = i + 1;
                                if ~ischar(varargin{i}),
                                    error('TRG:GraspSolver:NonConsistentValue', ...
                                        'Wrong value specified for ''%s'', expected a string.', ...
                                            currentProp);
                                end
                                if size(varargin{i}, 1) > 1,
                                    error('TRG:GraspSolver:NonConsistentValue', ...
                                        'Wrong value specified for ''%s'': expected a string, not an array of strings.', ...
                                            currentProp);
                                end
                                obj.(currentProp) = varargin{i};
                            case 'reducedProb',
                                i = i + 1;
                                if ~isa(varargin{i}, 'GrOpt2dProblem'),
                                    error('TRG:GraspSolver:NonConsistentValue', ...
                                        'Wrong value specified for ''reducedProb'', expected a GrOpt2dProblem object.');
                                end
                                obj.pReducedProb = varargin{i}.copy;
                            case 'PlotOpt',
                                i = i + 1;
                                if ~isa(varargin{i}, 'GrOptPlot'),
                                   error('TRG:EntityWithPOIs:NonConsistentValue', ...
                                        'Wrong value specified for ''PlotOpt'', expected a GrOptPlot object.');
                                end
                                obj.pPlotOpt = varargin{i}.copy;
                            otherwise,
                                i = i + 1;
                                % ignore properties that may be specific of subclasses
                        end
                        i = i + 1;
                    end
                end
                % if the problem is 2D (reducedProb.isSet == true), check
                % the consistency between the GrOpt2dProblem settings
                % and the contact models, and collect the indices of
                % force/velocity components to be ignored in the analysis
                %
                % (note that, if GrOpt2dProblem settings were already available for HandWithGlove,
                % and ObjectWithCover, these data structs themselves already performed the
                % consistency check for the POI transforms)
                %
                if obj.isProblem2d,
                    % check for the existence of SoftFingerContacts
                    % because the problem cannot be reduced to 2D in this case
                    cm = obj.contactStructure.contactModels;
                    if any(cellfun('isclass', num2cell(cm), 'SoftFingerContact')),
                        error('TRG:GraspSolver:NonConsistentValue', ...
                            'Impossible to reduce the problem to 2d, at least one contact is a SoftFingerContact.');
                    end
                    [modelIsConsistent, idx] = obj.pReducedProb.checkConsistencyAndSetIgnoredComponents({cm.contactFrameObjectSide});
                    if ~modelIsConsistent,
                        error('TRG:GraspSolver:NonConsistentValue', ...
                            'Impossible to reduce the problem to 2d, contact frames %d do not lie onto plane %s.',...
                            mat2str(idx), obj.pReducedProb.plane);
                    end
                    % now set their .isSet property to false (the settings
                    % in 'this' object are the MASTERS!
                    obj.handWithGlove.reducedProb = GrOpt2dProblem('isSet', false);
                    obj.objectWithCover.reducedProb = GrOpt2dProblem('isSet', false);
                end
            end
        end

        function rp = getReducedProbDeepCopy(obj)
            rp = obj.pReducedProb.copy;
        end

        %  wrapping the settings of a 2d problem  %

        function res = isProblem2d(obj)
            res = obj.pReducedProb.isSet;
        end

        %  set/get methods  %

        function set.name(obj, aString)
            if ~ischar(aString),
                error('TRG:GraspSolver:NonConsistentValue', ...
                	'Wrong value specified for ''name'', expected a string.');
            end
            if size(aString, 1) > 1,
                error('TRG:GraspSolver:NonConsistentValue', ...
                	'Wrong value specified for ''name'': expected a string, not an array of strings.');
            end
            obj.name = aString;
        end

        function set.comment(obj, aString)
            if ~ischar(aString),
                error('TRG:GraspSolver:NonConsistentValue', ...
                	'Wrong value specified for ''comment'', expected a string.');
            end
            if size(aString, 1) > 1,
                error('TRG:GraspSolver:NonConsistentValue', ...
                	'Wrong value specified for ''comment'': expected a string, not an array of strings.');
            end
            obj.comment = aString;
        end

        %  Dependent  Props %

        function rp = get.reducedProb(obj) %#ok<STOUT,MANU>
            error('TRG:GraspSolver:NonPermittedOperation', ...
                'Use ''getReducedProbDeepCopy()'' to get a copy of the GrOpt2dProblem object, or use ''isProblem2d()'' to know its current settings about 2D.');
        end

        function set.reducedProb(obj, rp)
            if ~isa(rp, 'GrOpt2dProblem'),
                error('TRG:GraspSolver:NonConsistentValue', ...
                        'Wrong value specified for ''reducedProb'', expected a GrOpt2dProblem object.');
            end
            obj.pReducedProb = rp.copy;
            % check consistency if 2D
            if obj.isProblem2d,
                cm = obj.contactStructure.contactModels;
                if any(cellfun('isclass', num2cell(cm), 'SoftFingerContact')),
                    error('TRG:GraspSolver:NonConsistentValue', ...
                        'Impossible to reduce the problem to 2d, at least one contact is a SoftFingerContact.');
                end
                [modelIsConsistent, idx] = obj.pReducedProb.checkConsistencyAndSetIgnoredComponents({cm.contactFrameObjectSide});
                if ~modelIsConsistent,
                    error('TRG:GraspSolver:NonConsistentValue', ...
                        'Impossible to reduce the problem to 2d, contact frames %d do not lie onto plane %s.',...
                        mat2str(idx), obj.pReducedProb.plane);
                end
            end
        end

        function set.PlotOpt(obj, po)
            if ~isa(po, 'GrOptPlot'),
                error('TRG:EntityWithPOIs:NonConsistentValue', ...
                    'Wrong value specified for ''PlotOpt'', expected a GrOptPlot object.');
            end
            obj.pPlotOpt = po.copy;
        end

        function po = get.PlotOpt(obj)
            po = obj.pPlotOpt;
        end

    end

end