% POIStructure. Abstract class for POI (Points of Interest) Data Structures
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

classdef POIStructure < handle

    properties (SetAccess = protected)
        nPois;
    end
    properties
        pois;
    end

    methods (Abstract)
        obj = addPOI(obj);
    end
    methods
        function display(obj)
            %POIStructure.display Display parameters
            %
            % aPOIStructure.display() displays the POIStructure parameters in human-readable form.
            %
            % NOTE:
            % - this method is invoked implicitly at the command line when the result 
            %   of an expression is a POIStructure object and the command has no trailing
            %   semicolon.
            %
            % See also SerialLink.display, Limb.display, Hand.display, POI.display, POIStructure.char.
            disp(' ');
            disp([inputname(1),' = '])
            disp(' ');
            for i = 1:length(obj),
                disp(char(char(obj(i)), ' '));
            end
            %disp(' ');
        end

        function s = char(obj)
            s = char(sprintf('  number of POIs: %d', obj.nPois), '');
            t = char('| numb. |  name');
            u = '';
            for i=1:obj.nPois,
                u = char(u, sprintf('|   %d   | %s', i, obj.pois(i).name));
            end
            uc = size(u,2);
            v = ['+-------+', char(45*ones(1,uc-8)), '+'];
            x = char(v, t, v, u(2:end,:), v);
            x([2,4:end-1],uc+2) = '|';
            s = char(s, x);
        end

        function removePOI(obj, i)
            if obj.nPois == 1,
                error('GRASP:classProps:NonConsistentValue', ...
                    'cannot have POIStructure objects with empty POI list');
            end
            if ~isnumumeric(i),
                error('GRASP:classProps:NonConsistentValue', ...
                    'wrong input argument: expected a vector of indices');
            end
            if any(i <= 0),
                error('GRASP:classProps:NonConsistentValue', ...
                    ['POI indices ', num2str(find(i <= 0)), 'are <= 0']);
            end
            if any(i > obj.nPois),
                error('GRASP:classProps:NonConsistentValue', ...
                    ['POI indices ', num2str(find(i > obj.nPois)), 'are out of bound (', num2str(obj.nPois)]);
            end
            obj.pois(real(i)) = [];
            obj.nPois = obj.nPois - length(i);
        end
    end

end