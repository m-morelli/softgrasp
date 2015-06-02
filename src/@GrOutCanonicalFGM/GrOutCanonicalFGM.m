% GrOutCanonicalFGM. Data Structure Encapsulating Results of the Canonical FGM Matrix Computation for Compliant Grasps
% With Postural Synergies
%   Description, in progress
%
%   Annotations, in progress
%
%   References, in progress
%
%   Matteo Morelli, I.R.C. "E. Piaggio", University of Pisa
%   Edoardo Farnioli, I.R.C. "E. Piaggio", University of Pisa
%
%   September 2013

% Copyright (C) 2013 Interdepartmental Research Center "E. Piaggio", University of Pisa
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

classdef GrOutCanonicalFGM < handle

    properties (Access = protected)
        cFGMstruct;
        I_idx;
        Adi_idx;
        f_block_idx;
        u_block_idx;
        d_we_idx;
        d_last_idx;
    end

    properties (Dependent)
        complete;
        cutIdentityBlock;
    end

    methods
        function obj = GrOutCanonicalFGM(varargin)
            obj.cFGMstruct = [];
            obj.I_idx = [];
            obj.Adi_idx = [];
            obj.f_block_idx = [];
            obj.u_block_idx = [];
            obj.d_we_idx = [];
            obj.d_last_idx = [];
            if nargin > 1,
                % varargin
                if ~strcmp(varargin{1},'cFGMstruct'),
                    error('TRG:GrOutCanonicalFGM:NonConsistentValue', ...
                        'Wrong input argument, first input is not the FGMstruct (??!!).');
                end
                obj.cFGMstruct = varargin{2};
                i = 3;
                while i < length(varargin),
                    processedProp = varargin{i};
                    switch processedProp,
                        case {'I_idx', 'Adi_idx', 'f_block_idx', 'u_block_idx', 'd_we_idx', 'd_last_idx'},
                            currentProp = varargin{i};
                            i = i + 1;
                            if ~isa(varargin{i}, 'numeric'),
                                error('TRG:GrOutCanonicalFGM:UnknownProperty', ...
                                    'Wrong input argument, result is not numeric (??!!).');
                            end
                            obj.(currentProp) = varargin{i};
                        otherwise,
                    end
                    i = i + 1;
                end
            end
        end

        function cFGM = get.complete(obj)
            cFGM = obj.cFGMstruct.FGM;
        end

        function Adi = get.cutIdentityBlock(obj)
            Adi = obj.cFGMstruct.FGM(:, obj.Adi_idx);
        end
    end
end