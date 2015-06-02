% CompleteConstraintContact. Concrete implementation of Contact objects encapsulating the functionalities of Complete
% Constraints Contacts
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
%       - using base class constructor/copy
%       - error checking
%       - removed p* Constant properties

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

classdef CompleteConstraintContact < Contact

    properties (Access=protected, Constant)
        transmittedDofs3d = 6;
        forceMomentTransmittedDofs3d = [3, 3];
        transmittedDofs2d = 3;
        forceMomentTransmittedDofs2d = [2, 1];
    end
    properties (Constant)
        typeFormatLong = 'complete-constraint';
    end

    methods (Access=protected)
        function [cstr_i_lin, cstr_i_rot] = structuralCompliance2d(obj)
            cstr_i_lin = obj.pStructuralCompliance(1:2);
            cstr_i_rot = 0;
        end
    end
    methods
        function obj = CompleteConstraintContact(varargin)
            % super()
            obj@Contact(varargin{:});

            % contact specific default initializations
            if ~(length(obj) > 1),
                if isempty(obj.pStructuralCompliance),
                    obj.pStructuralCompliance = [20, 20, 20, ... % translational (Nmm)
                                                100, 100, 100];  % torsional     (degree)
                end
            end
        end

        function obj2 = copy(obj)
            obj2 = CompleteConstraintContact(obj);
        end

        function fs = forceSelector(obj, reducedProb) %#ok<MANU>
            s = 3;
            if nargin > 1,
                if ~isa(reducedProb, 'GrOpt2dProblem'),
                    error('TRG:CompleteConstraintContact:NonConsistentValue', ...
                        'Wrong value specified for ''reducedProb'', expected a GrOpt2dProblem object.');
                end
                if reducedProb.isSet,
                    s = 2;
                end
            end
            fs = eye(s);
        end

        function ms = momentSelector(obj, reducedProb) %#ok<MANU>
            d = 6; s = 3;
            if nargin > 1,
                if ~isa(reducedProb, 'GrOpt2dProblem'),
                    error('TRG:CompleteConstraintContact:NonConsistentValue', ...
                        'Wrong value specified for ''reducedProb'', expected a GrOpt2dProblem object.');
                end
                if reducedProb.isSet,
                    d = 3; s = 2;
                end
            end
            ms = eye(d-s);
        end

        function msb = momentSelectorBase(obj, varargin)
            narginchk(2,3);
            varargin(1) = []; % do not consider u for CompleteConstraintContacts
            msb = momentSelector(obj, varargin{:});
        end
    end

end
