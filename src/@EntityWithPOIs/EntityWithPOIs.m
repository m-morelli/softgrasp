% EntityWithPOIs. Abstract class for Objects (Entities) Having POIs (Points of Interest Data Structures)
%   Description, in progress
%
%   Annotations, in progress
%
%   References, in progress
%
%   Matteo Morelli, I.R.C. "E. Piaggio", University of Pisa
%
%   May 2012
%   Modified on September 2013
%       - general function clean-up
%       - reducedProb (GrOpt2dProblem) settings
%       - PlotOpt (GrOptPlot) settings

% Copyright (C) 2012, 2013 Interdepartmental Research Center "E. Piaggio", University of Pisa
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

classdef EntityWithPOIs < handle

    properties
        name;
        comment;
    end

    properties (Dependent)
        reducedProb;
    end

    properties (Access = protected)
        pReducedProb;
    end
    
    properties (SetAccess = protected)
        POIStructure;
        PlotOpt;
    end

    methods (Abstract)
        T = poiForwardKinematics(obj, eConf);
        Dmb = poiDifferentialMapBase(obj, eConf);
        Dmt = poiDifferentialMapTool(obj, eConf);
        plot(obj, eConf);
    end

    methods
        function obj = EntityWithPOIs(varargin)
            % defaults
            obj.name = 'noname';
            obj.comment = '';
            obj.pReducedProb = GrOpt2dProblem;
            obj.POIStructure = [];
            obj.PlotOpt = GrOptPlot;

            % process required user-providen options
            if nargin < 1,
                error('MATLAB:narginchk:notEnoughInputs', 'Not enough input arguments.');
            end
            arg1 = varargin{1};
            if isa(arg1, 'EntityWithPOIs'),
                % Copy
                if numel(arg1) ~= 1,
                    error('TRG:EntityWithPOIs:NonConsistentValue',...
                        'Wrong input argument: expected a EntityWithPOIs, not an array of EntityWithPOIs.');
                end
                obj.name = arg1.name;
                obj.comment = arg1.comment;
                obj.pReducedProb = arg1.pReducedProb.copy;
                obj.POIStructure = arg1.POIStructure.copy;
                obj.PlotOpt = arg1.PlotOpt.copy;
            else
                % check POIStruct
                if ~isa(arg1, 'POIStructure'),
                    error('TRG:EntityWithPOIs:NonConsistentValue',...
                            'Wrong input argument, expected a POIStructure object.');
                end
                if numel(arg1) ~= 1,
                    error('TRG:EntityWithPOIs:NonConsistentValue',...
                            'Wrong input argument: expected a POIStructure, not an array of POIStructures.');
                end
                obj.POIStructure = arg1.copy;
                %
                % now perform a check for optional user-providen options
                if nargin > 1,
                    if nargin < 3,
                        error('MATLAB:narginchk:notEnoughInputs', 'Not enough input arguments.');
                    end
                    i = 2;
                    while i < nargin,
                        processedProp = varargin{i};
                        if ~isa(processedProp, 'char'),
                            error('TRG:EntityWithPOIs:NonConsistentValue', ...
                                'Expected the name of an option.');
                        end
                        switch processedProp,
                            case {'name', 'comment'},
                                currentProp = varargin{i};
                                i = i + 1;
                                if ~ischar(varargin{i}),
                                    error('TRG:EntityWithPOIs:NonConsistentValue', ...
                                        'Wrong value specified for ''%s'', expected a string.', ...
                                            currentProp);
                                end
                                if size(varargin{i}, 1) > 1,
                                    error('TRG:EntityWithPOIs:NonConsistentValue', ...
                                        'Wrong value specified for ''%s'': expected a string, not an array of strings.', ...
                                            currentProp);
                                end
                                obj.(currentProp) = varargin{i};
                            case 'reducedProb',
                                i = i + 1;
                                if ~isa(varargin{i}, 'GrOpt2dProblem'),
                                    error('TRG:EntityWithPOIs:NonConsistentValue', ...
                                        'Wrong value specified for ''reducedProb'', expected a GrOpt2dProblem object.');
                                end
                                obj.pReducedProb = varargin{i}.copy;
                            case 'PlotOpt',
                                i = i + 1;
                                if ~isa(varargin{i}, 'GrOptPlot'),
                                   error('TRG:EntityWithPOIs:NonConsistentValue', ...
                                        'Wrong value specified for ''PlotOpt'', expected a GrOptPlot object.');
                                end
                                obj.PlotOpt = varargin{i}.copy;
                            otherwise,
                                i = i + 1;
                                % ignore properties that may be specific of subclasses
                        end
                        i = i + 1;
                    end
                end
                % if the problem is 2D (reducedProb.isSet == true), check
                % the consistency between the GrOpt2dProblem settings
                % and the POI transforms, and collect the indices of
                % force/velocity components to be ignored in the analysis
                if obj.isProblem2d,
                    pois = obj.POIStructure.pois;
                    [modelIsConsistent, idx] = obj.pReducedProb.checkConsistencyAndSetIgnoredComponents({pois.relativeTransform});
                    if ~modelIsConsistent,
                        error('TRG:EntityWithPOIs:NonConsistentValue', ...
                            'Impossible to reduce the problem to 2d, POI frames %d do not lie onto plane %s.',...
                                mat2str(idx), obj.pReducedProb.plane);
                    end
                end
            end
        end

        function display(obj)
            disp(' ');
            disp([inputname(1),' = '])
            disp(' ');
            for i = 1:length(obj),
                disp(char(char(obj(i)), ' '));
            end
        end

        function s = char(obj)
            s = char('', sprintf('          POIs: %d', obj.POIStructure.nPois));
        end

        %  wrapping the settings of a 2d problem  %

        function res = isProblem2d(obj)
            res = obj.pReducedProb.isSet;
        end

        function rp = getReducedProbDeepCopy(obj)
            rp = obj.pReducedProb.copy;
        end

        %  set/get methods  %

        function set.name(obj, aString)
            if ~ischar(aString),
                error('TRG:EntityWithPOIs:NonConsistentValue', ...
                	'Wrong value specified for ''name'', expected a string.');
            end
            if size(aString, 1) > 1,
                error('TRG:EntityWithPOIs:NonConsistentValue', ...
                	'Wrong value specified for ''name'': expected a string, not an array of strings.');
            end
            obj.name = aString;
        end

        function set.comment(obj, aString)
            if ~ischar(aString),
                error('TRG:EntityWithPOIs:NonConsistentValue', ...
                	'Wrong value specified for ''comment'', expected a string.');
            end
            if size(aString, 1) > 1,
                error('TRG:EntityWithPOIs:NonConsistentValue', ...
                	'Wrong value specified for ''comment'': expected a string, not an array of strings.');
            end
            obj.comment = aString;
        end

        %  Dependent  Props %

        function rp = get.reducedProb(obj) %#ok<STOUT,MANU>
            error('TRG:EntityWithPOIs:NonPermittedOperation', ...
                'Cannot get a GrOpt2dProblem object, use ''isProblem2d()'' for getting info about the 2D setting of a EntityWithPOIs Object.');
        end

        function set.reducedProb(obj, rp)
            if ~isa(rp, 'GrOpt2dProblem'),
                error('TRG:EntityWithPOIs:NonConsistentValue', ...
                        'Wrong value specified for ''reducedProb'', expected a GrOpt2dProblem object.');
            end
            obj.pReducedProb = rp.copy;
            % check consistency if 2D
            if obj.isProblem2d,
                pois = obj.POIStructure.pois;
                [modelIsConsistent, idx] = obj.pReducedProb.checkConsistencyAndSetIgnoredComponents({pois.relativeTransform});
                if ~modelIsConsistent,
                    error('TRG:EntityWithPOIs:NonConsistentValue', ...
                        'Impossible to reduce the problem to 2d, POI frames %s do not lie onto plane %s.',...
                            mat2str(idx), obj.pReducedProb.plane);
                end
            end
        end

%{
        function set.POIStructure(obj, poiStr)
            if ~isa(poiStr, 'POIStructure'),
                error('TRG:EntityWithPOIs:NonConsistentValue', ...
                    'Wrong value specified for ''POIStructure'', expected a POIStructure object.');
            end
            if numel(poiStr) ~= 1,
                error('TRG:EntityWithPOIs:NonConsistentValue', ...
                    'Wrong input argument: expected a POIStructure, not an array of POIStructures.');
            end
            obj.POIStructure = poiStr;
        end
%}
    end

end