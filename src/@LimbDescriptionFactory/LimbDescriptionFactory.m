% LimbDescriptionFactory. LimbDescriptionFactory Class
%   Description, in progress
%
%   Annotations, in progress
%
%   References, in progress
%
%   Matteo Morelli, I.R.C. "E. Piaggio", University of Pisa
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
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
%
% You should have received a copy of the GNU General Public License
% along with THE Robotic Grasping Toolbox for use with MATLAB(R).  If not, see <http://www.gnu.org/licenses/>.

classdef LimbDescriptionFactory < handle

    methods (Static)

        function obj = create(varargin)
            if nargin < 1,
                error('MATLAB:narginchk:notEnoughInputs', 'Not enough input arguments.');
            end
            id = varargin{1};
            if ~isa(id, 'char'),
                error('TRG:create:NonConsistentValue', 'Wrong input argument, expected a string (unique identifier).');
            end
            switch id,
                case 'screw',
                    obj = LimbDescriptionScrew(varargin{2:end});
                case 'dh',
                    obj = LimbDescriptionDH(varargin{2:end});
                case 'urdf',
                    obj = LimbDescriptionURDF(varargin{2:end});
                otherwise,
                    error('TRG:create:UnknownProperty', 'Unknown Limb kinematic description.');
            end
            % automatic registration:
            % - uid_name.conf (MyDescriptionConstructor)
            % - replace switch ... end with the following (pseudo-)code:
            %   - find(uid_name.conf)
            %   - if ~exists then error ('Unknown Limb kinematic description.')
            %     else
            %       - read the file (MyDescriptionConstructor)
            %       - execute the string
        end

    end
    
end