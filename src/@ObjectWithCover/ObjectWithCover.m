% ObjectWithCover. Concrete implementation of EntityWithPOIs objects modelling a ManipulatedObject With POIs
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
%       - code clean-up
%       - using reducedProb & PlotOpt
%   Modified on November 2013: renamed from Cover to ObjectWithCover

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
    
classdef ObjectWithCover < EntityWithPOIs

    properties (SetAccess = protected)
        object;
    end

    methods
        function obj = ObjectWithCover(varargin)
            % process required user-providen options
            if nargin < 1,
                error('MATLAB:narginchk:notEnoughInputs', 'Not enough input arguments.');
            end
            if nargin == 1,
                % Copy
                arg1 = varargin{1};
                if ~isa(arg1, 'ObjectWithCover'),
                    error('TRG:ObjectWithCover:NonConsistentValue', ...
                        'Wrong input argument, expected a ObjectWithCover.');
                end
                if numel(arg1) ~= 1,
                    error('TRG:ObjectWithCover:NonConsistentValue', ...
                        'Wrong input argument: expected a ObjectWithCover, not an array of ObjectWithCover objects.');
                end
                object = arg1.object;
                oplOther = {arg1}; % super() will perform the copy of the other properties
            else
                % nargin > 1
                oplOther = varargin(2:end);
                object = varargin{1};
            end

            % super()
            %
            % set POIStructure (required!)
            % and name, comment, GrOpt2dProblem, PlotOpt (if any)
            obj@EntityWithPOIs(oplOther{:});

            % assign the object
            obj.object = object.copy;

            % check if ManipulatedObject & ObjectPOIStrucure (to be done only when not copying)
            if nargin > 1,
                if ~isa(object, 'ManipulatedObject'),
                    error('TRG:ObjectWithCover:NonConsistentValue', ...
                        'Wrong input argument, expected a ManipulatedObject.');
                end
                if numel(object) ~= 1,
                    error('TRG:ObjectWithCover:NonConsistentValue', ...
                        'Wrong input argument: expected a ManipulatedObject, not an array of ManipulatedObjects.');
                end
                % obj.object (= object.copy) has right type and dimensions.
                %
                % check if the POIStructure (assigned in super()) is
                % actually a Cover
                if ~isa(obj.POIStructure, 'Cover'),
                    error('TRG:ObjectWithCover:NonConsistentValue', ...
                        'Wrong input argument, expected a Cover.');
                end
                if numel(obj.POIStructure) ~= 1,
                    error('TRG:ObjectWithCover:NonConsistentValue', ...
                        'Wrong input argument: expected a Cover, not an array of Cover objects.');
                end
                % obj.POIStructure has right type and dimensions.
            end
        end

        function obj2 = copy(obj)
            obj2 = ObjectWithCover(obj);
        end

        function s = char(obj)
            x = obj.object.char();
            t = char@EntityWithPOIs(obj);
            sAttach = ', attached on the object, at %s';
            s_attach = '';
            nPois = obj.POIStructure.nPois;
            for i = 1:nPois,
                s_attach = char(s_attach, sprintf(...
                    sAttach, ...
                        mat2str(transl(obj.POIStructure.pois(i).relativeTransform)')));
            end
            s_Sp = char(32*ones(nPois, 4));
            s_cTypeLongInfo = char(obj.POIStructure.pois.name);
            z = [s_Sp, strcat(s_cTypeLongInfo, s_attach(2:end,:))];
            s = char(x, t, z);
        end
    end
    
end