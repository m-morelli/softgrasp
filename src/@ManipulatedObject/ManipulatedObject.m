% ManipulatedObject. Manipulated Object class
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

classdef ManipulatedObject < handle

    properties (SetAccess = private)
        handle;
    end
    properties
        name;
        manuf;
        comment;
        radius;
        height;
    end

    methods
        function obj = ManipulatedObject(varargin)
            %ManipulatedObject Create a ManipulatedObject object
            %
            % SYNOPSIS
            %
            % OPTIONS
            %
            % NOTE:
            %  - ManipulatedObject is a reference object, a subclass of Handle object.
            %  - ManipulatedObject objects can be used in vectors and arrays
            %
            % See also ManipulatedObjectWithPointsOfInterest.

            % default props
            obj.name = 'noname';
            obj.manuf = '';
            obj.comment = '';
            obj.handle = [];

            % process user-providen props
            if nargin > 0,
                switch(nargin),
                    case 1,
                        % ManipulatedObject
                        if ~isa(varargin{1}, 'ManipulatedObject'),
                            error('GRASP:classProps:UnknownInput', ...
                                'wrong input argument, expected a ManipulatedObject');
                        end
                        if length(varargin{1}) ~= 1,
                            error('GRASP:classProps:NonConsistentValue', ...
                                'wrong input argument: expected a ManipulatedObject, not an array of ManipulatedObjects');
                        end
                        meta = ?ManipulatedObject;
                        metaPropName = {meta.PropertyList.Name};
                        for p = 1:length(metaPropName),
                            currentProp = metaPropName{p};
                            try
                                eval(['obj.', currentProp, ...
                                    ' = varargin{1}.', currentProp, ';']);
                            catch ME,
                                %warning(ME.identifier, ME.message);
                            end
                        end
                    otherwise,
                        % varargin
                        i = 1;
                        while i < length(varargin),
                            processedProp = varargin{i};
                            switch processedProp,
                                case {'name', 'manuf', 'comment'},     
                                    currentProp = varargin{i};
                                    i = i + 1;
                                    if ~ischar(varargin{i}),
                                        error('GRASP:classProps:NonConsistentValue', ...
                                            'wrong value specified for ''%s'', expected a string', ...
                                                currentProp);
                                    end
                                    if size(varargin{i}, 1) > 1,
                                        error('GRASP:classProps:NonConsistentValue', ...
                                            'wrong value specified for ''%s'': expected a string, not an array of strings', ...
                                                currentProp);
                                    end
                                    eval(['obj.', currentProp, ' = varargin{i};']);
                                    
                                case {'radius', 'height',},     %%%%%%% SEZIONE AGGIUNTA
                                    currentProp = varargin{i};
                                    i = i + 1;
                                    if ~isnumeric(varargin{i}),
                                        error('GRASP:classProps:NonConsistentValue', ...
                                            'wrong value specified for ''%s'', expected a numeric value', ...
                                                currentProp);
                                    end
                                    if size(varargin{i}, 1) > 1,
                                        error('GRASP:classProps:NonConsistentValue', ...
                                            'wrong value specified for ''%s'': expected a numeric value, not an array', ...
                                                currentProp);
                                    end
                                    eval(['obj.', currentProp, ' = varargin{i};']);
                                    
                                    %%% FINE AGGIUNTA
                                otherwise,
                                    error('GRASP:classProps:UnknownProperty', ...
                                        'unknown property passed to ManipulatedObject <%s>', processedProp);
                            end
                            i = i + 1;
                        end
                end
            end
        end

        function obj2 = copy(obj)
            %ManipulatedObject.copy Clone a ManipulatedObject object
            %
            % aManipulatedObject2 = aManipulatedObject.copy() is a deepcopy of the object aManipulatedObject.
            obj2 = ManipulatedObject(obj);
        end

        function display(obj)
            %ManipulatedObject.display Display parameters
            %
            % aManipulatedObject.display() displays the ManipulatedObject parameters in human-readable form.
            %
            % NOTE:
            % - this method is invoked implicitly at the command line when the result 
            %   of an expression is a ManipulatedObject object and the command has no trailing
            %   semicolon.
            %
            % See also ManipulatedObjectWithPointsOfInterest.display, ManipulatedObjectWithPointsOfInterest.char.
            disp(' ');
            disp([inputname(1),' = '])
            disp(' ');
            for i = 1:length(obj),
                disp(char(char(obj(i)), ' '));
            end
        end

        function s = char(obj)
            %ManipulatedObject.char String representation of parameters
            %
            % S = aManipulatedObject.char() is a string representation of the ManipulatedObject parameters.
            s_descr = obj.name;
            if ~isempty(obj.manuf),
                s_descr = [s_descr, ', [', obj.manuf, ']'];
            end
            if ~isempty(obj.comment),
                s_descr = [s_descr, ', <', obj.comment, '>'];
            end
            s =         sprintf('   manip. object: %s', s_descr);
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

        function set.manuf(obj, aString)
            if ~ischar(aString),
                error('GRASP:classProps:NonConsistentValue', ...
                	'wrong value specified for ''manuf'', expected a string');
            end
            if size(aString, 1) > 1,
                error('GRASP:classProps:NonConsistentValue', ...
                	'wrong value specified for ''manuf'': expected a string, not an array of strings');
            end
            obj.manuf = aString;
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
        
        function set.radius(obj, val)
            if isa(val, 'numeric')
                if(size(val,1)~= 1 || size(val,2)~= 1)
                    error('GRASP:classProps:NonConsistentValue', ...
                    'wrong value specified for radius, expected a single numeric value, not an array');
                end
            else
                error('GRASP:classProps:NonConsistentValue', ...
                'wrong value specified for radius, expected a numeric value ');
            end
            obj.radius = abs(val);
            
        end
        
        function set.height(obj, val)
            if isa(val, 'numeric')
                if(size(val,1)~= 1 || size(val,2)~= 1)
                    error('GRASP:classProps:NonConsistentValue', ...
                    'wrong value specified for height, expected a single numeric value, not an array');
                end
            else
                error('GRASP:classProps:NonConsistentValue', ...
                'wrong value specified for height, expected a numeric value ');
            end
            obj.height = abs(val);
            
        end
    end

end