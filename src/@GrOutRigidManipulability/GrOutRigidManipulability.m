% GrOutRigidManipulability. Data Structure Encapsulating Results of Manipolability Subspaces Computation for Rigid
% Grasps Without Synergies
%   Description, in progress
%
%   Annotations, in progress
%
%   References, in progress
%
%   Matteo Morelli, I.R.C. "E. Piaggio", University of Pisa
%
%   June 2012

% Copyright (C) 2012 Interdepartmental Research Center "E. Piaggio", University of Pisa
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

classdef GrOutRigidManipulability < handle

    properties (SetAccess = protected)
        nw;
        na;
        nh;
        nc;
        ns;
        gmh;
        gmfh;
        gmw;
        gmt;
        gmf;
        gms;
        gmfs;
    end

    methods
        function obj = GrOutRigidManipulability(varargin)
            obj.nw = [];
            obj.na = [];
            obj.nh = [];
            obj.nc = [];
            obj.ns = [];
            obj.gmh = [];
            obj.gmfh = [];
            obj.gmw = [];
            obj.gmt = [];
            obj.gmf = [];
            obj.gms = [];
            obj.gmfs = [];
            if nargin > 1,
                % varargin
                i = 1;
                while i < length(varargin),
                    processedProp = varargin{i};
                    switch processedProp,
                        case {'gmh', 'gmfh', 'gmw', 'gmt', ...
                              'gmf', 'gms', 'gmfs', ...
                              'nw', 'na', 'nh', 'nc', 'ns'},
                            currentProp = varargin{i};
                            i = i + 1;
                            if ~isa(varargin{i}, 'numeric'),
                                error('GRASP:classProps:UnknownProperty', ...
                                    'wrong input argument, result is not numeric (??!!)');
                            end
                            eval(['obj.', currentProp, ' = varargin{i};']);
                        otherwise,
                    end
                    i = i + 1;
                end
            end
        end

        function [nw, na, nh, nc, ns, gmh, gmfh, gmw, gmt, gmf, gms, gmfs] = getAll(obj)
            nw = obj.nw;
            na = obj.na;
            nh = obj.nh;
            nc = obj.nc;
            ns = obj.ns;
            gmh = obj.gmh;
            gmfh = obj.gmfh;
            gmw = obj.gmw;
            gmt = obj.gmt;
            gmf = obj.gmf;
            gms = obj.gms;
            gmfs = obj.gmfs;
        end

        function [gmx] = getWholeSubspace(obj)
            gmx = [ zeros(obj.nw,obj.nh), obj.gmw, obj.gms; ...
                                 obj.gmh, obj.gmt, zeros(obj.na,obj.ns); ...
                                obj.gmfh, obj.gmf, obj.gmfs             ];
        end
    end
end