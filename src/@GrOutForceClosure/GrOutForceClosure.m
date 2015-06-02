% GrOutForceClosure. Data Structure Encapsulating Results of GFO
%   Description, in progress
%
%   Annotations, in progress
%
%   References, in progress
%
%   Matteo Morelli, I.R.C. "E. Piaggio", University of Pisa
%
%   May 2012

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

classdef GrOutForceClosure < handle

    properties (SetAccess = protected)
        y;
        v;
        nSteps;
        log;
    end
    properties (Dependent)
        yStar;
        vStar;
        logStar;
    end

    methods
        function obj = GrOutForceClosure(varargin)
            obj.y = [];
            obj.v = [];
            obj.nSteps = 0;
            obj.log = '';
            if nargin > 1,
                % varargin
                i = 1;
                while i < length(varargin),
                    processedProp = varargin{i};
                    switch processedProp,
                        case {'y', 'v', 'nSteps'},
                            currentProp = varargin{i};
                            i = i + 1;
                            if ~isa(varargin{i}, 'numeric'),
                                error('TRG:GrOutForceClosure:UnknownProperty', ...
                                    'Wrong input argument, result is not numeric (??!!).');
                            end
                            obj.(currentProp) = varargin{i};
                        case 'log',
                            i = i + 1;
                            if ~ischar(varargin{i}),
                                error('TRG:GrOutForceClosure:NonConsistentValue', ...
                                    'Wrong input argument, expected a string (??!!).');
                            end
                            obj.log = varargin{i};
                        otherwise,
                    end
                    i = i + 1;
                end
            end
        end

        function [y, v, nSteps, log] = getAll(obj)
            y = obj.y;
            v = obj.v;
            nSteps = obj.nSteps;
            log = obj.log;
        end

        function val = get.yStar(obj)
            val = obj.y(:,end);
        end

        function val = get.vStar(obj)
            val = obj.v(:,end);
        end

        function val = get.logStar(obj)
            val = obj.log(end, :);
        end
    end
end