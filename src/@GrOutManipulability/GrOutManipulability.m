% GrOutManipulability. Data Structure Encapsulating Results of Manipulability Subspaces Computation for Compliant Grasps
% With Postural Synergies
%   Description, in progress
%
%   Annotations, in progress
%
%   References, in progress
%
%   Matteo Morelli, I.R.C. "E. Piaggio", University of Pisa
%
%   June 2012
%   Modified on August 2013
%       - The data struct is now based on the FGM formulation
%         and it allows users to address the problem of computing
%         the manip. indices according to [1]

% Copyright (C) 2012, 2013 Interdepartmental Research Center "E. Piaggio", University of Pisa
%
% This file is part of THE Robotic Grasping Toolbox for use with MATLAB(R).
%
% THE Robotic Grasping Toolbox for use with MATLAB(R) is free software: you can redistribute it and/or modify it under
% the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the
% License, or (at your option) any later version.
%
% THE Robotic Grasping Toolbox for use with MATLAB(R) is distributed in the hope that it will be useful, but WITHOUT ANY
% WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
% General Public License for more details.
%
% You should have received a copy of the GNU General Public License along with THE Robotic Grasping Toolbox for use with
% MATLAB(R).  If not, see <http://www.gnu.org/licenses/>.

classdef GrOutManipulability < handle

    properties (Access = protected)
        FGMstruct;
    end
    properties (SetAccess = protected)
        ni;
        nsq;
        nk;
        nst;
        nco;
        gm_i;
        gm_sq;
        gm_k;
        gm_st;
        gm_co;
    end

    methods
        function obj = GrOutManipulability(varargin)
            obj.FGMstruct = [];
            obj.ni = [];
            obj.nsq = [];
            obj.nk = [];
            obj.nst = [];
            obj.nco = [];
            obj.gm_i = [];
            obj.gm_sq = [];
            obj.gm_k = [];
            obj.gm_st = [];
            obj.gm_co = [];
            if nargin > 1,
                % varargin
                if ~strcmp(varargin{1},'FGMstruct'),
                    error('TRG:GrOutManipulability:NonConsistentValue', ...
                        'Wrong input argument, first input is not the FGMstruct (??!!).');
                end
                obj.FGMstruct = varargin{2};
                i = 3;
                while i < length(varargin),
                    processedProp = varargin{i};
                    switch processedProp,
                        case {'ni',     'nsq',   'nk',   'nst',   'nco'},
                            currentProp = varargin{i};
                            i = i + 1;
                            if ~isa(varargin{i}, 'numeric'),
                                error('TRG:GrOutManipulability:UnknownProperty', ...
                                    'Wrong input argument, result is not numeric (??!!).');
                            end
                            obj.(currentProp) = varargin{i};
                        case {'gm_i',   'gm_sq', 'gm_k', 'gm_st', 'gm_co'},
                            currentProp = varargin{i};
                            i = i + 1;
                            if ~(isempty(varargin{i}) || isa(varargin{i}, 'struct')),
                                error('TRG:GrOutManipulability:UnknownProperty', ...
                                    'Wrong input argument, result is not a structure array (??!!).');
                            end
                            if ~isempty(varargin{i}),
                                obj.(currentProp) = varargin{i};
                            else
                                obj.(currentProp) = cell2struct( cell(length(obj.FGMstruct.conf),1), obj.FGMstruct.conf );
                            end
                        otherwise,
                    end
                    i = i + 1;
                end
            end
        end
    end
end