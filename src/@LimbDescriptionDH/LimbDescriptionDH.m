% LimbDescriptionDH. LimbDescriptionDH Class
%   Description, in progress
%
%   Annotations, in progress
%
%   References, in progress
%
%   Matteo Morelli, I.R.C. "E. Piaggio", University of Pisa
%
%   May 2012
%   Changed to LimbDescriptionDH in July 2013

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

classdef LimbDescriptionDH < LimbDescription

    methods (Access = private)

        toScrew(objScrew, objDH);

    end

    methods

        function obj = LimbDescriptionDH(varargin)
            %
            % SYNOPSYS
            % l2 = LimbDescriptionDH(l1)
            % l1 = LimbDescriptionDH('PropertyName', PropertyValue, ...)
            % l1 = LimbDescriptionDH(links, 'PropertyName', PropertyValue, ...)
            % l1 = LimbDescriptionDH(DH, 'PropertyName', PropertyValue, ...)
            %
            % References: RTB by Peter Corke (CSIRO).

            % process user-providen options
            if nargin < 1,
                error('MATLAB:narginchk:notEnoughInputs', 'Not enough input arguments.');
            end
            arg1 = varargin{1};
            if isa(arg1, 'LimbDescriptionDH'),
                if nargin > 1,
                    error('MATLAB:narginchk:tooManyInputs', 'Too many input arguments.');
                end
                if length(arg1) ~= 1,
                    error('TRG:LimbDescriptionDH:NonConsistentValue', 'Wrong input argument: expected a LimbDescriptionDH, not an array of LimbDescriptionDH objects.');
                end
                % deep-copy
                obj.n = arg1.n;
                obj.P = arg1.P;
                obj.Z = arg1.Z;
                obj.g_st0 = arg1.g_st0;
                obj.joints = arg1.joints;
            else
                % RTB()
                tmpSerial = SerialLink(varargin{:});
                % convert to Screw represent.
                obj.toScrew(tmpSerial);
            end

        end

        function obj2 = copy(obj1)
            %LimbDescriptionDH.copy Clone a LimbDescriptionDH object
            %
            % l2 = l1.copy() is a deepcopy of the object l1.
            obj2 = LimbDescriptionDH(obj1);
        end

    end
    
end