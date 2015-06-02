% GrOutRigidMobility. Data Structure Encapsulating Results of Mobility Subspaces Computation for Rigid
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

classdef GrOutRigidMobility < handle

    properties (SetAccess = protected)
        nu;
        na;
        np;
        nr;
        nc;
        ni;
        gmar;
        gmac;
        gmpac;
        gmpoac;
        gmopac;
        gmoac;
        gmpr;
        gmpoi;
        gmopi;
        gmoi;
    end

    methods
        function obj = GrOutRigidMobility(varargin)
            obj.nu = [];
            obj.na = [];
            obj.np = [];
            obj.nr = [];
            obj.nc = [];
            obj.ni = [];
            obj.gmar = [];
            obj.gmac = [];
            obj.gmpac = [];
            obj.gmpoac = [];
            obj.gmopac = [];
            obj.gmoac = [];
            obj.gmpr = [];
            obj.gmpoi = [];
            obj.gmopi = [];
            obj.gmoi = [];
            if nargin > 1,
                % varargin
                i = 1;
                while i < length(varargin),
                    processedProp = varargin{i};
                    switch processedProp,
                        case {'gmar',  'gmac', 'gmpac', 'gmpoac', 'gmopac', ...
                              'gmoac', 'gmpr', 'gmpoi', 'gmopi',  'gmoi', ...
                              'nu',    'na',   'np',    'nr',     'nc', 'ni'},
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

        function res = gmapc(obj)
            res = obj.gmac(:, 1:obj.nc(1));
        end

        function res = gmapoc(obj)
            res = obj.gmac(:, obj.nc(1)+1:obj.nc(2));
        end

        function res = gmaoc(obj)
            res = obj.gmac(:, obj.nc(2)+1:obj.nc(3));
        end

        function [nu, na, np, nr, nc, ni, gmar, gmapc, gmapoc, gmaoc, gmpac, gmpoac, gmopac, gmoac, gmpr, gmpoi, gmopi, gmoi] = getAll(obj)
            nu = obj.nu;
            na = obj.na;
            np = obj.np;
            nr = obj.nr;
            nc = obj.nc;
            ni = obj.ni;
            gmar = obj.gmar;
            gmapc = obj.gmapc;
            gmapoc = obj.gmapoc;
            gmaoc = obj.gmaoc;
            gmpac = obj.gmpac;
            gmpoac = obj.gmpoac;
            gmopac = obj.gmopac;
            gmoac = obj.gmoac;
            gmpr = obj.gmpr;
            gmpoi = obj.gmpoi;
            gmopi = obj.gmopi;
            gmoi = obj.gmoi;
        end

        function [gmx] = getWholeSubspace(obj)
            gmx = [     obj.gmar,                        obj.gmac,                                 zeros(obj.na,sum(obj.ni)); ...
                 zeros(obj.np,obj.nr), [obj.gmpac, obj.gmpoac, zeros(obj.np,obj.nc(3))], [obj.gmpr, obj.gmpoi, zeros(obj.np,obj.ni(3))]; ...
                 zeros(obj.nu,obj.nr), [zeros(obj.nu,obj.nc(1)), obj.gmopac, obj.gmoac], [zeros(obj.nu,obj.ni(1)), obj.gmopi, obj.gmoi]];
        end
    end
end