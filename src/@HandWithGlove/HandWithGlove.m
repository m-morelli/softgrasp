% HandWithGlove. Concrete implementation of EntityWithPOIs objects modelling a Hand With POIs
%   Description, in progress
%
%   Annotations, in progress
%
%   References, in progress
%
%   Matteo Morelli, I.R.C. "E. Piaggio", University of Pisa
%
%   May 2012
%   Modified on July 2013: poiDifferentialMap is now a protected method
%   Modified on November 2013: name changed from Glove to HandWithGlove

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

classdef HandWithGlove < EntityWithPOIs

    properties (SetAccess = protected)
        hand;
    end

    methods
        function obj = HandWithGlove(varargin)
            % process required user-providen options
            if nargin < 1,
                error('MATLAB:narginchk:notEnoughInputs', 'Not enough input arguments.');
            end
            if nargin == 1,
                % Copy
                arg1 = varargin{1};
                if ~isa(arg1, 'HandWithGlove'),
                    error('TRG:HandWithGlove:NonConsistentValue', ...
                        'Wrong type of 1st argument, expected a HandWithGlove.');
                end
                if numel(arg1) ~= 1,
                    error('TRG:HandWithGlove:NonConsistentValue', ...
                        'Wrong input argument: expected a HandWithGlove, not an array of HandWithGlove objects.');
                end
                hnd = arg1.hand;
                oplOther = {arg1}; % super() will perform the copy of the other properties
            else
                % nargin > 1
                oplOther = varargin(2:end);
                hnd = varargin{1};
            end

            % super()
            %
            % set POIStructure (required!)
            % and name, comment, GrOpt2dProblem, PlotOpt (if any)
            obj@EntityWithPOIs(oplOther{:});

            % assign the hand
            obj.hand = hnd.copy;

            % check if Hand & HandPOIStrucure (to be done only when not copying)
            if nargin > 1,
                if ~isa(hnd, 'Hand'),
                    error('TRG:HandWithGlove:NonConsistentValue', ...
                        'Wrong input argument, expected a Hand.');
                end
                if numel(hnd) ~= 1,
                    error('TRG:HandWithGlove:NonConsistentValue', ...
                        'Wrong input argument: expected a Hand, not an array of Hands.');
                end
                % obj.hand (= hnd.copy) has right type and dimensions.
                %
                % check if the POIStructure (assigned in super()) is
                % actually a Glove
                if ~isa(obj.POIStructure, 'Glove'),
                    error('TRG:HandWithGlove:NonConsistentValue', ...
                        'Wrong input argument, expected a Glove.');
                end
                if numel(obj.POIStructure) ~= 1,
                    error('TRG:HandWithGlove:NonConsistentValue', ...
                        'Wrong input argument: expected a Glove, not an array of Glove objects.');
                end
                % obj.POIStructure has right type and dimensions.
                %
                % ? check if attach indices are ok. ?
            end
        end

        function obj2 = copy(obj)
            obj2 = HandWithGlove(obj);
        end

        function s = char(obj)
            x = obj.hand.char();
            t = char@EntityWithPOIs(obj);
            sAttach = {', attached on the palm', ... on an element in the world', ...
                       ', attached on link %d of limb %d, at %s'};
            s_attach = '';
            nPois = obj.POIStructure.nPois;
            for i = 1:nPois,
                s_attach = char(s_attach, sprintf(...
                    sAttach{~isempty(obj.POIStructure.attachIndices{i}) + 1}, ...
                        obj.POIStructure.attachIndices{i}, ...
                            mat2str(transl(obj.POIStructure.pois(i).relativeTransform)')));
            end
            s_Sp = char(32*ones(nPois, 4));
            s_cTypeLongInfo = char(obj.POIStructure.pois.name);
            z = [s_Sp, strcat(s_cTypeLongInfo, s_attach(2:end,:))];
            s = char(x, t, z);
        end
    end

    methods (Access = protected)
        [M, m_cell] = poiDifferentialMap(obj, q, jCbk, rp);

        basePlot(obj, q, PlotOpt);
    end

end