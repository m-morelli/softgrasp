% SoftFingerContact. Concrete implementation of Contact objects encapsulating the functionalities of Soft Finger
% Contacts
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

classdef SoftFingerContact < Contact

    properties (Access=protected, Constant)
        transmittedDofs3d = 4;
        forceMomentTransmittedDofs3d = [3, 1];
        transmittedDofs2d = [];
        forceMomentTransmittedDofs2d = [];
    end
    properties (Constant)
        typeFormatLong = 'soft-finger';
    end

    methods (Access=protected)
        function [cstr_i_lin, cstr_i_rot] = structuralCompliance2d(obj) %#ok<STOUT>
            error('TRG:SoftFingerContact:NonConsistentValue', ...
                'Unexpected SoftFingerContact object for a 2D problem! Please report this fault.');
        end
    end
    methods
        function obj = SoftFingerContact(varargin)
            % super()
            obj@Contact(varargin{:});

            % contact specific default initializations
            if ~(length(obj) > 1),
                if isempty(obj.pStructuralCompliance),
                    obj.pStructuralCompliance = [20, 20, 20, ... % translational (Nmm)
                                                100];            % torsional     (degree)
                end
            end
        end

        function obj2 = copy(obj)
            obj2 = SoftFingerContact(obj);
        end

        function fs = forceSelector(obj, reducedProb)
            narginchk(1,2);
            s = 3;
            if nargin > 1,
                % this is actually due to the need to have
                % the interface of forceSelector() congruent
                % with HardFingerContacts and CompleteConstraintContacts
                %
                % However, in no cases a GrOpt2dProblem with isSet == true
                % shall be used with soft-fingers!
                if ~isa(reducedProb, 'GrOpt2dProblem'),
                    error('TRG:SoftFingerContact:NonConsistentValue', ...
                        'Wrong value specified for ''reducedProb'', expected a GrOpt2dProblem object.');
                end
                if reducedProb.isSet,
                    error('TRG:SoftFingerContact:NonConsistentValue', ...
                        'Unexpected SoftFingerContact object for a 2D problem! Please report this fault.');
                end
            end
            fs = eye(s);
        end

        function ms = momentSelector(obj, reducedProb_) %#ok<*MANU>
            narginchk(1,2);
            if nargin > 1,
                % this is actually due to the need to have
                % the interface of forceSelector() congruent
                % with HardFingerContacts and CompleteConstraintContacts
                %
                % However, in no cases a GrOpt2dProblem with isSet == true
                % shall be used with soft-fingers!
                if ~isa(reducedProb_, 'GrOpt2dProblem'),
                    error('TRG:SoftFingerContact:NonConsistentValue', ...
                        'Wrong value specified for ''reducedProb'', expected a GrOpt2dProblem object.');
                end
                if reducedProb_.isSet,
                    if reducedProb.isSet,
                        error('TRG:SoftFingerContact:NonConsistentValue', ...
                            'Unexpected SoftFingerContact object for a 2D problem! Please report this fault.');
                    end
                end
            end
            ms = [0,0,1];
        end

        function msb = momentSelectorBase(obj, u, reducedProb_)
            narginchk(2,3);
            if nargin > 2,
                % this is actually due to the need to have
                % the interface of forceSelector() congruent
                % with HardFingerContacts and CompleteConstraintContacts
                %
                % However, in no cases a GrOpt2dProblem with isSet == true
                % shall be used with soft-fingers!
                if ~isa(reducedProb_, 'GrOpt2dProblem'),
                    error('TRG:SoftFingerContact:NonConsistentValue', ...
                        'Wrong value specified for ''reducedProb'', expected a GrOpt2dProblem object.');
                end
                if reducedProb_.isSet,
                    if reducedProb.isSet,
                        error('TRG:SoftFingerContact:NonConsistentValue', ...
                            'Unexpected SoftFingerContact object for a 2D problem! Please report this fault.');
                    end
                end
            end
            if ~isa(u, 'numeric'),
                error('TRG:SoftFingerContact:UnknownProperty', ...
                    'Wrong 2nd input argument, expected an object configuration vector (u).');
            end
            u = real(u(:));
            % check conf size
            if numel(u) ~= 6,
                error('TRG:SoftFingerContact:NonConsistentValue', ...
                    'Wrong object config, expected 6 elements in place of %d.', ...
                        numel(u));
            end
            ri = eul2r(u(4:6)');
            cx = ri*tr2rt(obj.contactFrameObjectSide);
            msb = (cx(1:3,3))';
        end
    end

end
