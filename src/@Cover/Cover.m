% Cover. Concrete implementation of POIStructure for use with ManipulatedObject instances
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

classdef Cover < POIStructure

    methods
        function obj = Cover(varargin)
            % process user-providen props
            if nargin > 0,
                switch(class(varargin{1})),
                    case 'Cover',
                        if length(varargin{1}) ~= 1,
                            error('GRASP:classProps:NonConsistentValue', ...
                                'wrong input argument: expected a Cover, not an array of Cover objects');
                        end
                        meta = ?Cover;
                        metaPropName = {meta.PropertyList.Name};
                        for p = 1:length(metaPropName),
                            currentProp = metaPropName{p};
                            try
                                eval(['obj.', currentProp, ...
                                    ' = varargin{1}.', currentProp, ';']);
                            catch ME,
                                warning(ME.identifier, ME.message);
                            end
                        end
                    case 'POI',
                        if nargin > 1,
                            error('GRASP:classProps:NonConsistentValue', ...
                                'wrong number of rhs arguments, expected 1');
                        end
                        poi = varargin{1}(:);
                        poiL = numel(poi);
                        if areThereMorePOIsAtSameLocation(poi),
                            error('GRASP:classProps:NonConsistentValue', ...
                                'detected multiple POIs at the same location');
                        end
                        obj.pois = poi;
                        obj.nPois = poiL;
                    %case 'char',
                    otherwise,
                        error('GRASP:classProps:UnknownProperty', ...
                            'unknown property passed to Cover');
                end
            else
                error('GRASP:classProps:NonConsistentValue', ...
                    'cannot create a Cover without POI objects');
            end
        end

        function obj2 = copy(obj)
            obj2 = Cover(obj);
        end

        function s = char(obj)
            s = char@POIStructure(obj);
            %{
            latestCharInS = find(s(end,:) == '+', 1, 'last');
            s_top = s(1:2,:);
            s_low = s(3:end,1:latestCharInS);
            t =         '-----------+-------------+';
            t = char(t, ' on phal.# | of finger # |');
            t = char(t, '-----------+-------------+');
            for i=1:length(obj.pois),
                t = char(t, sprintf('     %d     |      %d      |', obj.pAttachIndices(i,:)));
            end
            t = char(t, '-----------+-------------+');
            s = char(s_top, [s_low, t]);
            %}
        end

        function obj = addPOI(obj, poi)
            if nargin ~= 2,
                error('GRASP:classProps:NonConsistentValue', ...
                    'wrong number of rhs arguments, expected 1');
            end
            if ~isa(poi, 'POI'),
                error('GRASP:classProps:UnknownProperty', ...
                    'wrong input argument, expected a POI');
            end
            poi = poi(:); poiL = length(poi);
            if areThereMorePOIsAtSameLocation(poi, obj.pois),
                error('GRASP:classProps:NonConsistentValue', ...
                    'detected multiple POIs at the same location');
            end
            obj.pois = [obj.pois; poi];
            obj.nPois = obj.nPois + poiL;
        end
    end

end

%
%
%
function mpBool = areThereMorePOIsAtSameLocation(poi, opois_)
    mpBool = 0;
    poiL = length(poi);
    opois = [];
    positionTestCommandStr = 'cellfun(@(x) all(abs(transl(poi(i).relativeTransform)'' - transl(x)'') < 1e-6), {poi(i+1:end,:).relativeTransform}, ''UniformOutput'', true)';
    if nargin > 1,
        opois = opois_;
        mpBool = morePOIsAtSameLocationCore(poiL, poi, positionTestCommandStr, []);
        positionTestCommandStr = 'cellfun(@(x) all(abs(transl(poi(i).relativeTransform)'' - transl(x)'') < 1e-6), {opois.relativeTransform}, ''UniformOutput'', true)';
    end
    mpBool = any([mpBool, morePOIsAtSameLocationCore(poiL, poi, positionTestCommandStr, opois)]);
end

function mpBool = morePOIsAtSameLocationCore(poiL, poi, positionTestCommandStr, opois)
    mpBool = 0;
    for i = 1:poiL - isempty(opois),
        mpTr = eval(positionTestCommandStr);
        if any(mpTr),
            mpBool = 1;
            warning('GRASP:classProps:NonConsistentValue', ...
                ['trying to add POI ''', poi(i).name, ''' (', num2str(i),...
                ') at the same location as POIs ', sprintf('%d, ', find(mpTr) + isempty(opois)*i)]);
        end
    end
end