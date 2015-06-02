% Hand. Hand Class
%   Description, in progress
%
%   Annotations, in progress
%
%   References, in progress
%
%   Matteo Morelli, I.R.C. "E. Piaggio", University of Pisa
%
%   May 2012
%   Modified in July 2013
%       - maintainance

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

classdef Hand < handle

    properties (SetAccess = protected)
        limbs;
        nLimbs;
    end
    properties
        name;
        manuf;
        comment;
        palm;
        %synergies;
    end

    methods
        function obj = Hand(arg1, varargin)
            %Hand Create a Hand object
            %
            % SYNOPSIS
            %
            % OPTIONS
            %
            % NOTE:
            %  - Hand is a reference object, a subclass of Handle object.
            %  - Hand objects can be used in vectors and arrays
            %
            % See also Link, SerialLink, Limb, Hand.plot.

            % default props
            obj.name = 'noname';
            obj.manuf = '';
            obj.comment = '';
            obj.palm = eye(4);
            obj.limbs = [];
            obj.nLimbs = 0;
            %obj.synergies = [];

            % process user-providen props
            if nargin > 0,
                if isa(arg1, 'Hand'),
                    if length(arg1) ~= 1,
                    	error('TRG:Hand:NonConsistentValue', 'Wrong input argument: expected a Hand, not an array of Hand objects.');
                    end
                    % deep-copy
                    obj.name = arg1.name;
                    obj.manuf = arg1.manuf;
                    obj.comment = arg1.comment;
                    obj.palm = arg1.palm;
                    obj.limbs = arg1.limbs.copy;
                    obj.nLimbs = arg1.nLimbs;
                    %obj.synergies = arg1.synergies;
                elseif isa(arg1, 'Limb'),
                    if ~any(size(arg1) == 1),
                    	error('TRG:Hand:NonConsistentValue', 'Wrong input argument: expected an array of Limbs, not a matrix of Limbs.');
                    end
                    obj.limbs = arg1;
                    obj.nLimbs = length(arg1);
                    %obj.synergies = eye(obj.dofs);
                    % check if limb names are uniques
                    [uqNames, idxUqNames] = unique(obj.limbNames);
                    idxNonUqNames = 1:obj.nLimbs; idxNonUqNames(idxUqNames) = [];
                    for i = idxNonUqNames,
                    	warning('TRG:Hand:MultipleDefs', 'Found a duplicate for name ''%s'', going to change it.', obj.limbs(i).name);
                        newName = [obj.limbs(i).name, '_', num2str(i)];
                        while any(strcmp(newName, uqNames)),
                        	newName = [newName, '_', num2str(i)]; %#ok<AGROW>
                        end
                        obj.limbs(i).name = newName;
                    end
                else
                	error('TRG:Hand:UnknownInput', 'Wrong input argument, expected a Hand or an array of Limbs.');
                end
                if nargin > 1,
                    % varargin
                    i = 1;
                    while i < length(varargin),
                        processedProp = varargin{i};
                        switch processedProp,
                            case {'name', 'manuf', 'comment'},
                                currentProp = varargin{i};
                                i = i + 1;
                                if ~ischar(varargin{i}),
                                    error('TRG:Hand:NonConsistentValue', ...
                                        'Wrong value specified for ''%s'', expected a string.', ...
                                            currentProp);
                                end
                                if size(varargin{i}, 1) > 1,
                                    error('TRG:Hand:NonConsistentValue', ...
                                        'Wrong value specified for ''%s'': expected a string, not an array of strings.', ...
                                            currentProp);
                                end
                                eval(['obj.', currentProp, ' = varargin{i};']);
                            case 'palm',
                                i = i + 1;
                                if ~ishomog(varargin{i}),
                                    error('TRG:Hand:NonConsistentValue', ...
                                        'Wrong value specified for ''palm'', expected a homogeneous transform.');
                                end
                                obj.palm = varargin{i};
%{
                            case 'synergies',
                                i = i + 1;
                                if ~isnumeric(varargin{i}),
                                    error('TRG:Hand:NonConsistentValue', ...
                                        'Wrong value specified for ''synergies'', expected a numeric matrix.');
                                end
                                if size(varargin{i}, 1) ~= obj.dofs,
                                    error('TRG:Hand:NonConsistentValue', ...
                                        'Wrong value specified for ''synergies'', expected a numeric matrix with %d rows.', ...
                                            obj.dofs);
                                end
                                obj.synergies = varargin{i};
%}
                            otherwise,
                                error('TRG:Hand:UnknownProperty', ...
                                    'Unknown property passed to Hand ''%s''.', processedProp);
                        end
                        i = i + 1;
                    end
                end
            end
        end

        function obj2 = copy(obj)
            %Hand.copy Clone a Hand object
            %
            % aHand2 = aHand.copy() is a deepcopy of the object aHand.
            obj2 = Hand(obj);
        end

        function display(obj)
            %Hand.display Display parameters
            %
            % aHand.display() displays the Hand parameters in human-readable form.
            %
            % NOTE:
            % - this method is invoked implicitly at the command line when the result 
            %   of an expression is a Hand object and the command has no trailing
            %   semicolon.
            %
            % See also SerialLink.display, Limb.display, Hand.char.
            disp(' ');
            disp([inputname(1),' = '])
            disp(' ');
            for i = 1:length(obj),
                disp(char(char(obj(i)), ' '));
            end
        end

        function s = char(obj)
            %Hand.char String representation of parameters
            %
            % S = aHand.char() is a string representation of the Hand parameters.
            s_descr = obj.name;
            if ~isempty(obj.manuf),
                s_descr = [s_descr, ', [', obj.manuf, ']'];
            end
            if ~isempty(obj.comment),
                s_descr = [s_descr, ', <', obj.comment, '>'];
            end
            s =         sprintf('   hand system: %s', s_descr);
            s = char(s, '');
            s = char(s, [char('            P = ', ' ', ' ', ' '), num2str(obj.palm)]);
            s = char(s, '');
            s = char(s, sprintf('         limbs: %d', obj.nLimbs));
            if obj.nLimbs > 0,
                s_Sp = char(32*ones(obj.nLimbs, 4));
                limbNames = char(obj.limbNames);
                limbDofs = num2str(obj.dofsPerLimb');
                s_sp = char(32*ones(obj.nLimbs, 1));
                limbConfigs = char(obj.limbs.config);
                s = char(s, [s_Sp, limbNames, strcat(' (', limbDofs, ' axes,', [s_sp, limbConfigs], ')')]);
            end
        end

        function v = dofs(obj)
            v = 0;
            if ~isempty(obj.limbs),
                v = sum([obj.limbs.n]);
            end
        end

        function v = dofsPerLimb(obj)
            v = [obj.limbs.n];
        end

%{
        function v = hasSynergisticCouplings(obj)
            v = ~isequal(obj.synergies, eye(size(obj.synergies)));
        end

        function v = synergisticDofs(obj)
            v = 0;
            if obj.hasSynergisticCouplings,
                v = size(obj.synergies,2);
            end
        end
%}

        function v = limbNames(obj)
            v = {obj.limbs.name};
        end

        % set/get methods

        function set.name(obj, aString)
            if ~ischar(aString),
                error('TRG:Hand:NonConsistentValue', ...
                	'Wrong value specified for ''name'', expected a string.');
            end
            if size(aString, 1) > 1,
                error('TRG:Hand:NonConsistentValue', ...
                	'Wrong value specified for ''name'': expected a string, not an array of strings.');
            end
            obj.name = aString;
        end

        function set.manuf(obj, aString)
            if ~ischar(aString),
                error('TRG:Hand:NonConsistentValue', ...
                	'Wrong value specified for ''manuf'', expected a string.');
            end
            if size(aString, 1) > 1,
                error('TRG:Hand:NonConsistentValue', ...
                	'Wrong value specified for ''manuf'': expected a string, not an array of strings.');
            end
            obj.manuf = aString;
        end

        function set.comment(obj, aString)
            if ~ischar(aString),
                error('TRG:Hand:NonConsistentValue', ...
                	'Wrong value specified for ''comment'', expected a string.');
            end
            if size(aString, 1) > 1,
                error('TRG:Hand:NonConsistentValue', ...
                	'Wrong value specified for ''comment'': expected a string, not an array of strings.');
            end
            obj.comment = aString;
        end

        function set.palm(obj, transform)
            if ~ishomog(transform),
            	error('TRG:Hand:NonConsistentValue', ...
                	'Wrong value specified for ''palm'', expected a homogeneous transform.');
            end
            obj.palm = transform;
        end

%{
        function set.synergies(obj, synMatrix)
            if ~isnumeric(synMatrix),
                error('TRG:Hand:NonConsistentValue', ...
                	'Wrong value specified for ''synergies'', expected a numeric matrix.');
            end
            if size(synMatrix, 1) ~= obj.dofs,
                error('TRG:Hand:NonConsistentValue', ...
                	'Wrong value specified for ''synergies'', expected a numeric matrix with %d rows.', ...
                    	obj.dofs);
            end
            obj.synergies = synMatrix;
        end
%}

    end

    methods (Access = protected)

        conf = basePlot(obj, q, PlotOpt);

    end

end