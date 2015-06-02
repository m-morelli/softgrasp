% Contact. Abstract class for Contacts
%   Description, in progress
%
%   Annotations, in progress
%
%   References, in progress
%
%   Matteo Morelli, I.R.C. "E. Piaggio", University of Pisa
%
%   May 2012
%   Modified on July 2013
%       - removed u as input argument of force/moment Selector methods
%         (force/moment selector contributes are now computed according to
%         the contact frame)
%       - added a momentSelectorBase method that takes u as input
%       - constructor/copy
%       - error checking

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

classdef Contact < matlab.mixin.Heterogeneous & handle

    properties (Access = protected)
        pStructuralCompliance;
    end
    properties (Abstract, Access=protected, Constant)
        transmittedDofs3d;
        forceMomentTransmittedDofs3d;
        transmittedDofs2d;
        forceMomentTransmittedDofs2d;
    end
    properties (Abstract, Constant)
        typeFormatLong;
    end
    properties
        frictionCoefficients;
        name;
        comment;
    end
    properties (Hidden)
        contactFrameHandSide;
        contactFrameObjectSide;
    end

    methods (Abstract, Access=protected)
        [cstr_i_lin, cstr_rot] = structuralCompliance2d(obj);
    end
    methods (Abstract)
        fs  = forceSelector(obj, reducedProb);
        ms  = momentSelector(obj, reducedProb);
        msb = momentSelectorBase(obj, u, reducedProb);
    end
    methods
        function obj = Contact(varargin)
            % defaults
            obj.name = 'noname';
            obj.comment = '';
            obj.frictionCoefficients = [1, ... % static frict. coeff
                                        1];    % rotational frict. coeff
            if nargin > 0,
                arg1 = varargin{1};
                if isa(arg1, 'Contact'),
                    % copy
                    narginchk(1,1);
                    [nr, nc] = size(arg1);
                    lLength = nr*nc;
                    if lLength == 1,
                        obj.pStructuralCompliance = arg1.pStructuralCompliance;
                        obj.frictionCoefficients = arg1.frictionCoefficients;
                        obj.name = arg1.name;
                        obj.comment = arg1.comment;
                        obj.contactFrameHandSide = arg1.contactFrameHandSide;
                        obj.contactFrameObjectSide = arg1.contactFrameObjectSide;
                    elseif any([nr,nc] == 1),
                        % copy an array
                        for i = 1:lLength,
                            obj(i) = arg1(i).copy; %#ok<AGROW>
                        end
                        if nr > 1, obj = obj'; end
                    else
                        error('TRG:Contact:NonConsistentValue', 'Wrong input argument: expected an array of Contacts (nx1), not a matrix of Contacts (nxm).');
                    end
                elseif isa(arg1, 'char'),
                    i = 1;
                    while i < length(varargin),
                        processedProp = varargin{i};
                        switch processedProp,
                            case {'name', 'comment'},
                                currentProp = varargin{i};
                                i = i + 1;
                                if ~ischar(varargin{i}),
                                    error('TRG:Contact:NonConsistentValue', ...
                                        'Wrong value specified for ''%s'', expected a string.', ...
                                        currentProp);
                                end
                                if size(varargin{i}, 1) > 1,
                                    error('TRG:Contact:NonConsistentValue', ...
                                        'Wrong value specified for ''%s'': expected a string, not an array of strings.', ...
                                        currentProp);
                                end
                                eval(['obj.', currentProp, ' = varargin{i};']);
                            case 'frictionCoefficients',
                                i = i + 1;
                                if ~isnumeric(varargin{i}),
                                    error('TRG:Contact:NonConsistentValue', ...
                                        'Wrong value specified for ''frictionCoefficients'', expected a 2-element vector.');
                                end
                                if numel(varargin{i}) ~= 2,
                                    error('TRG:Contact:NonConsistentValue', ...
                                        'Wrong value specified for ''frictionCoefficients'', expected a 2-element vector.');
                                end
                                obj.frictionCoefficients = varargin{i};
                            case 'structuralCompliance',
                                i = i + 1;
                                if ~isnumeric(varargin{i}),
                                    error('TRG:Contact:NonConsistentValue', ...
                                        'Wrong value specified for ''structuralCompliance'', expected a numeric vector.');
                                end
                                transDofs = obj.transmittedDofs;
                                if numel(varargin{i}) ~= transDofs,
                                    error('TRG:Contact:NonConsistentValue', ...
                                        'Wrong value specified for ''structuralCompliance'', expected a numeric vector with %d elements.', ...
                                            transDofs);
                                end
                                obj.pStructuralCompliance = varargin{i};
                            otherwise,
                                error('TRG:Contact:UnknownProperty', ...
                                    'Unknown property passed to Contact ''%s''', processedProp);
                        end
                        i = i + 1;
                    end
                else
                    error('TRG:Contact:NonConsistentValue', 'Wrong input argument, expected a Contact model or a list of properties for a Contact model.');
                end
            end
        end

        function display(obj)
            %Contact.display Display parameters
            %
            % aContact.display() displays the Contact parameters in human-readable form.
            %
            % NOTE:
            % - this method is invoked implicitly at the command line when the result 
            %   of an expression is a Contact object and the command has no trailing
            %   semicolon.
            %
            % See also SerialLink.display, Limb.display, Hand.display, Contact.char.
            disp(' ');
            disp([inputname(1),' = '])
            disp(' ');
            for i = 1:length(obj),
                disp(char(char(obj(i)), ' '));
            end
            %disp(' ');
        end

        function s = char(obj)
            %Contact.char String representation of parameters
            %
            % S = aContact.char() is a string representation of the Contact parameters.
            tDofs = obj.transmittedDofs;
            s = char(sprintf('   contact model: %s', obj.typeFormatLong), '');
            s = char(s, [char('           Cstr = ', ...
                            32*ones(tDofs - 1, 1)),             num2str(obj.structuralCompliance)]);
            s = char(s, '');
            s = char(s, sprintf('    frictCoeffs = %.2f(t) %.2f(r)', obj.frictionCoefficients));
        end

        % set/get methods

        function set.name(obj, aString)
            if ~ischar(aString),
                error('TRG:Contact:NonConsistentValue', ...
                	'Wrong value specified for ''name'', expected a string.');
            end
            if size(aString, 1) > 1,
                error('TRG:Contact:NonConsistentValue', ...
                	'Wrong value specified for ''name'': expected a string, not an array of strings.');
            end
            obj.name = aString;
        end

        function set.comment(obj, aString)
            if ~ischar(aString),
                error('TRG:Contact:NonConsistentValue', ...
                	'Wrong value specified for ''comment'', expected a string.');
            end
            if size(aString, 1) > 1,
                error('TRG:Contact:NonConsistentValue', ...
                	'Wrong value specified for ''comment'': expected a string, not an array of strings.');
            end
            obj.comment = aString;
        end

        function set.contactFrameHandSide(obj, transform)
            if ~(isempty(transform) || ishomog(transform)),
                error('TRG:Contact:NonConsistentValue', ...
                	'Wrong value specified for ''contactFrameHandSide'', expected a homogeneous transform.');
            end
            obj.contactFrameHandSide = transform;
        end

        function set.contactFrameObjectSide(obj, transform)
            if ~(isempty(transform) || ishomog(transform)),
                error('TRG:Contact:NonConsistentValue', ...
                	'Wrong value specified for ''contactFrameObjectSide'', expected a homogeneous transform.');
            end
            obj.contactFrameObjectSide = transform;
        end

        function set.frictionCoefficients(obj, fCoeffs)
            if ~isnumeric(fCoeffs),
            	error('TRG:Contact:NonConsistentValue', ...
                	'Wrong value specified for ''frictionCoefficients'', expected a 2-element vector.');
            end
            if numel(fCoeffs) ~= 2,
                error('TRG:Contact:NonConsistentValue', ...
                	'Wrong value specified for ''frictionCoefficients'', expected a 2-element vector.');
            end
            obj.frictionCoefficients = fCoeffs;
        end

        % special access methods for structuralCompliance
        %
        % set (it operates directly on the pStructuralCompliance properties
        function set.pStructuralCompliance(obj, structComp)
            if ~isnumeric(structComp),
                error('TRG:Contact:NonConsistentValue', ...
                    'Wrong value specified for ''structuralCompliance'', expected a numeric vector.');
            end
            transDofs = obj.transmittedDofs;
            if numel(structComp) ~= transDofs,
            	error('TRG:Contact:NonConsistentValue', ...
                    'Wrong value specified for ''structuralCompliance'', expected a numeric vector with %d elements.', ...
                        transDofs);
            end
            obj.pStructuralCompliance = structComp;
        end
        %
        % get method for structuralCompliance (needed because no more than one out arg
        % are allowed for get. methods)
        function [Cstr_i, Cstr_i_lin, Cstr_i_rot] = structuralCompliance(obj, reducedProb)
            narginchk(1,2);
            [fmt] = obj.forceMomentTransmittedDofs3d; ft = fmt(1); mt = fmt(2);
            cstr_i_lin = obj.pStructuralCompliance(1:ft);
            cstr_i_rot = obj.pStructuralCompliance(1+ft:mt+ft);
            if nargin > 1,
                if ~isa(reducedProb, 'GrOpt2dProblem'),
                    error('TRG:Contact:NonConsistentValue', ...
                        'Wrong value specified for ''reducedProb'', expected a GrOpt2dProblem object.');
                end
                if reducedProb.isSet,
                    [cstr_i_lin, cstr_i_rot] = obj.structuralCompliance2d;
                end
            end
            Cstr_i_lin = diag(cstr_i_lin);
            Cstr_i_rot = diag(cstr_i_rot);
            Cstr_i = diag([cstr_i_lin, cstr_i_rot]);
        end

        % other methods
        function td  = transmittedDofs(obj, reducedProb)
            narginchk(1,2);
            td_ = {obj.transmittedDofs3d}';
            if nargin > 1,
                if ~isa(reducedProb, 'GrOpt2dProblem'),
                    error('TRG:Contact:NonConsistentValue', ...
                        'Wrong value specified for ''reducedProb'', expected a GrOpt2dProblem object.');
                end
                if reducedProb.isSet,
                    td_ = {obj.transmittedDofs2d}';
                end
                if any(cellfun(@isempty, td_)),
                    error('TRG:Contact:NonConsistentValue', ...
                        'Cannot determine the number of transmitted DoFs, there are contacts with no 2d counterpart.');
                end
            end
            if numel(obj) == 1,
                td_ = cell2mat(td_);
            end
            td = td_;
        end

        function fmd = forceMomentTransmittedDofs(obj, reducedProb)
            narginchk(1,2);
            fmd_ = {obj.forceMomentTransmittedDofs3d}';
            if nargin > 1,
                if ~isa(reducedProb, 'GrOpt2dProblem'),
                    error('TRG:Contact:NonConsistentValue', ...
                        'Wrong value specified for ''reducedProb'', expected a GrOpt2dProblem object.');
                end
                if reducedProb.isSet,
                    fmd_ = {obj.forceMomentTransmittedDofs2d}';
                end
                if any(cellfun(@isempty, fmd_)),
                    error('TRG:Contact:NonConsistentValue', ...
                        'Cannot determine the number of transmitted DoFs, there are contacts with no 2d counterpart.');
                end
            end
            if numel(obj) == 1,
                fmd_ = cell2mat(fmd_);
            end
            fmd = fmd_;
        end

        function bi = selectionMatrix(obj, reducedProb)
            % checks
            narginchk(1,2);
            if nargin > 1,
                if ~isa(reducedProb, 'GrOpt2dProblem'),
                    error('TRG:Contact:NonConsistentValue', ...
                        'Wrong value specified for ''reducedProb'', expected a GrOpt2dProblem object.');
                end
            else
                reducedProb = GrOpt2dProblem;
            end
            % selection matrix (complete)
            bi = blkdiag(obj.forceSelector(reducedProb), obj.momentSelector(reducedProb));
            % now get rid of null rows
            bi(~any(bi,2), :) = [];
        end
    end

end
