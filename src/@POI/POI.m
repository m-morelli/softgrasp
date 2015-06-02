% POI. Point of Interest Data Object
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

classdef POI < handle

    properties (SetAccess = private)
        handle;
    end
    properties
        name;
        comment;
        relativeTransform;
    end

    methods
        function obj = POI(varargin)
            %POI Create a POI object
            %
            % SYNOPSIS
            %
            % OPTIONS
            %
            % NOTE:
            %  - POI is a reference object, a subclass of Handle object.
            %  - POI objects can be used in vectors and arrays
            %
            % See also Link, SerialLink, Limb, Hand.

            % default props
            obj.name = 'noname';
            obj.comment = '';
            obj.relativeTransform = eye(4);
            obj.handle = [];

            % process user-providen props
            if nargin > 0,
                if nargin > 1,
                    % POI ( T, <optionArgs> )
                    idx = 2;
                    if ishomog(varargin{1}),
                        obj.relativeTransform = varargin{1};
                    elseif isa(varargin{1}, 'char'),
                        idx = 1;
                    else
                        error('GRASP:classProps:NonConsistentValue', ...
                            'wrong first argument (''relativeTransform''), expected a homogeneous transform');
                    end
                    optionArgs = varargin(idx:nargin);
                    i = 1;
                    while i < length(optionArgs),
                        processedProp = optionArgs{i};
                        switch processedProp,
                            case {'name', 'comment'},
                                currentProp = optionArgs{i};
                                i = i + 1;
                                if ~ischar(optionArgs{i}),
                                    error('GRASP:classProps:NonConsistentValue', ...
                                        'wrong value specified for ''%s'', expected a string', ...
                                            currentProp);
                                end
                                if size(optionArgs{i}, 1) > 1,
                                    error('GRASP:classProps:NonConsistentValue', ...
                                        'wrong value specified for ''%s'': expected a string, not an array of strings', ...
                                            currentProp);
                                end
                                eval(['obj.', currentProp, ' = optionArgs{i};']);
                            case 'relativeTransform',
                                if ~ishomog(transform),
                                    error('GRASP:classProps:NonConsistentValue', ...
                                        'wrong value specified for ''relativeTransform'', expected a homogeneous transform');
                                end
                                obj.relativeTransform = transform;
                            otherwise,
                                error('GRASP:classProps:UnknownProperty', ...
                                    'unknown property passed to POI <%s>', processedProp);
                        end
                        i = i + 1;
                    end
                else
                    % nargin == 1,
                    % POI ( poi ) _OR_ POI ( T )
                    if isa(varargin{1}, 'POI'),
                        % case POI ( poi )
                        if length(varargin{1}) ~= 1,
                            error('GRASP:classProps:NonConsistentValue', ...
                                'wrong input argument: expected a POI, not an array of POIs');
                        end
                        meta = ?POI;
                        metaPropName = {meta.PropertyList.Name};
                        for p = 1:length(metaPropName),
                            currentProp = metaPropName{p};
                            try
                                eval(['obj.', currentProp, ...
                                    ' = varargin{1}.', currentProp, ';']);
                            catch ME,
                                warning(ME.identifier, ME.message);
                            end
                        end
                    elseif ishomog(varargin{1}),
                        obj.relativeTransform = varargin{1};
                    else
                        error('GRASP:classProps:NonConsistentValue', ...
                            'wrong input argument, expected a POI or a homogeneous transform');
                    end
                end
            end
        end

        function obj2 = copy(obj)
            %POI.copy Clone a POI object
            %
            % aPOI2 = aPOI.copy() is a deepcopy of the object aPOI.
            obj2 = POI(obj);
        end

        function display(obj)
            %POI.display Display parameters
            %
            % aPOI.display() displays the POI parameters in human-readable form.
            %
            % NOTE:
            % - this method is invoked implicitly at the command line when the result 
            %   of an expression is a POI object and the command has no trailing
            %   semicolon.
            %
            % See also SerialLink.display, Limb.display, Hand.display, POI.char.
            disp(' ');
            disp([inputname(1),' = '])
            disp(' ');
            for i = 1:length(obj),
                disp(char(char(obj(i)), ' '));
            end
            %disp(' ');
        end

        function s = char(obj)
            %POI.char String representation of parameters
            %
            % S = aPOI.char() is a string representation of the POI parameters.
            s_descr = obj.name;
            if ~isempty(obj.comment),
                s_descr = [s_descr, ', <', obj.comment, '>'];
            end
            s =         sprintf('   POI info: %s', s_descr);
            s = char(s, '');
            s = char(s, [char('        rT = ', ' ', ' ', ' '), num2str(obj.relativeTransform)]);
        end

        function set.name(obj, aString)
            if ~ischar(aString),
                error('GRASP:classProps:NonConsistentValue', ...
                	'wrong value specified for ''name'', expected a string');
            end
            if size(aString, 1) > 1,
                error('GRASP:classProps:NonConsistentValue', ...
                	'wrong value specified for ''name'': expected a string, not an array of strings');
            end
            obj.name = aString;
        end

        function set.comment(obj, aString)
            if ~ischar(aString),
                error('GRASP:classProps:NonConsistentValue', ...
                	'wrong value specified for ''comment'', expected a string');
            end
            if size(aString, 1) > 1,
                error('GRASP:classProps:NonConsistentValue', ...
                	'wrong value specified for ''comment'': expected a string, not an array of strings');
            end
            obj.comment = aString;
        end

        function set.relativeTransform(obj, transform)
            if ~ishomog(transform),
                error('GRASP:classProps:NonConsistentValue', ...
                	'wrong value specified for ''relativeTransform'', expected a homogeneous transform');
            end
            obj.relativeTransform = transform;
        end

    end

end