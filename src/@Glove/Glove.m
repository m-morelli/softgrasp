% Glove. Concrete implementation of POIStructure for use with Hand instances
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

classdef Glove < POIStructure

    properties (Access = private)
        pAttachIndices;
    end
    properties (SetAccess = private)
        attachIndices;
    end
    methods
        function obj = Glove(varargin)
            % process user-providen props
            if nargin > 0,
                switch(class(varargin{1})),
                    case 'Glove',
                        if length(varargin{1}) ~= 1,
                            error('GRASP:classProps:NonConsistentValue', ...
                                'wrong input argument: expected a Glove, not an array of Glove objects');
                        end
                        meta = ?Glove;
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
                        if nargin ~= 2,
                            error('GRASP:classProps:NonConsistentValue', ...
                                'wrong number of rhs arguments, expected 2');
                        end
                        poi = varargin{1}(:);
                        poiL = numel(poi);
                        if ~isa(varargin{2}, 'numeric'),
                            error('GRASP:classProps:UnknownProperty', ...
                                'wrong input argument, expected a 2-element row vector of attach indices ([phalange,finger])');
                        end
                        if ~all(size(varargin{2}) == [poiL,2])
                            error('GRASP:classProps:NonConsistentValue', ...
                                'wrong input argument: expected a 2-element row vector of attach indices ([phalange,finger])');
                        end
                        if ~isempty(find(varargin{2} < 0, 1)),
                            error('GRASP:classProps:NonConsistentValue', ...
                                'wrong input argument: found attach indices < 0');
                        end
                        attidx = real(varargin{2});
                        % POI on the palm?
                        zPh = find(attidx(:,1) == 0);
                        zFng = poiL + find(attidx(:,2) == 0);
                        if ~isempty(zPh),
                            attidx(zPh + poiL) = 0;
                        end
                        if ~isempty(zFng),
                            attidx(zFng - poiL) = 0;
                        end
                        if areThereMorePOIsAtSameLocation(poi, attidx),
                            error('GRASP:classProps:NonConsistentValue', ...
                                'detected multiple POIs at the same location');
                        end
                        obj.pois = poi;
                        obj.nPois = poiL;
                        obj.pAttachIndices = attidx;
                    %case 'char',
                    otherwise,
                        error('GRASP:classProps:UnknownProperty', ...
                            'unknown property passed to Glove');
                end
            else
                error('GRASP:classProps:NonConsistentValue', ...
                    'cannot create a Glove without POI objects');
            end
        end

        function obj2 = copy(obj)
            obj2 = Glove(obj);
        end

        function s = char(obj)
            s = char@POIStructure(obj);
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
        end

        function obj = addPOI(obj, poi, attidx)
            if nargin ~= 3,
                error('GRASP:classProps:NonConsistentValue', ...
                    'wrong number of rhs arguments, expected 2');
            end
            if ~isa(poi, 'POI'),
                error('GRASP:classProps:UnknownProperty', ...
                    'wrong input argument, expected a POI');
            end
            poi = poi(:); poiL = length(poi);
            if ~isa(attidx, 'numeric'),
                error('GRASP:classProps:UnknownProperty', ...
                    'wrong input argument, expected a 2-element row vector of attach indices ([phalange,finger])');
            end
            if ~all(size(attidx) == [poiL,2])
                error('GRASP:classProps:NonConsistentValue', ...
                    'wrong input argument: expected a 2-element row vector of attach indices ([phalange,finger])');
            end
            if ~isempty(find(attidx < 0, 1)),
                error('GRASP:classProps:NonConsistentValue', ...
                    'wrong input argument: found attach indices < 0');
            end
            % POI on the palm?
            zPh = find(attidx(:,1) == 0);
            zFng = poiL + find(attidx(:,2) == 0);
            if ~isempty(zPh),
                attidx(zPh + poiL) = 0;
            end
            if ~isempty(zFng),
                attidx(zFng - poiL) = 0;
            end
            if areThereMorePOIsAtSameLocation(poi, obj.pois, attidx, zeroCellOfAttachIndices(obj)),
            %if areThereMorePOIsAtSameLocation(poi, obj.pois, attidx, obj.pAttachIndices),
                error('GRASP:classProps:NonConsistentValue', ...
                    'detected multiple POIs at the same location');
            end
            attidx = real(attidx);
            obj.pois = [obj.pois; poi];
            obj.nPois = obj.nPois + poiL;
            obj.pAttachIndices = [obj.pAttachIndices; attidx];
        end

        function ai = get.attachIndices(obj)
            ai = cellfun(@(x) x(x>0), zeroCellOfAttachIndices(obj), 'UniformOutput',false);
        end
    end

end

%
%
%
function ai = zeroCellOfAttachIndices(obj)
	ai = mat2cell(obj.pAttachIndices, ones(1,size(obj.pAttachIndices,1)), 2);
end

function mpBool = areThereMorePOIsAtSameLocation(poi, opois_, attidx_, oais_)
    mpBool = 0;
    poiL = length(poi);
    attidx = opois_;
    opois = [];
    oais = [];
    placeTestCommandStr = '[logical(zeros(i,1)); cellfun(@(x) ismember(attidx(i,:),x,''rows''), mat2cell(attidx(i+1:end,:), ones(1,poiL-i), 2), ''UniformOutput'', true)]';
    %placeTestCommandStr = 'ismember(attidx(i,:),attidx(i+1:end,:))';
    positionTestCommandStr = 'cellfun(@(x) all(abs(transl(poi(i).relativeTransform)'' - transl(x)'') < 1e-6), {poi(mpIdx).relativeTransform}, ''UniformOutput'', true)';
    if nargin > 2,
        attidx = attidx_;
        opois = opois_;
        oais = oais_;
        mpBool = morePOIsAtSameLocationCore(poiL, poi, attidx, placeTestCommandStr, positionTestCommandStr, [], []);
        placeTestCommandStr = '[logical(zeros(i,1)); cellfun(@(x) ismember(attidx(i,:),x,''rows''), oais, ''UniformOutput'', true)]';
        positionTestCommandStr = 'cellfun(@(x) all(abs(transl(poi(i).relativeTransform)'' - transl(x)'') < 1e-6), {opois(mpIdx).relativeTransform}, ''UniformOutput'', true)';
    end
    mpBool = any([mpBool, morePOIsAtSameLocationCore(poiL, poi, attidx, placeTestCommandStr, positionTestCommandStr, opois, oais)]);
end

function mpBool = morePOIsAtSameLocationCore(poiL, poi, attidx, placeTestCommandStr, positionTestCommandStr, opois, oais) %#ok<INUSD,INUSL>
    mpBool = 0;
    for i = 1:poiL - isempty(opois),
        mpIdx = eval(placeTestCommandStr);
        if any(mpIdx),
            mpTr = eval(positionTestCommandStr);
            mpTrIdx = find(mpIdx);
            if any(mpTr),
                mpBool = 1;
                warning('GRASP:classProps:NonConsistentValue', ...
                    ['trying to add POI ''', poi(i).name, ''' (', num2str(i),...
                    ') at the same location as POIs ', sprintf('%d, ', mpTrIdx(mpTr) + isempty(opois)*i)]);
            end
        end
    end
end