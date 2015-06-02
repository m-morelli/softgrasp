% Limb. Limb Class
%   Description, in progress
%
%   Annotations, in progress
%
%   References, in progress
%
%   Matteo Morelli, I.R.C. "E. Piaggio", University of Pisa
%
%   July 2013

% Copyright (C) 2013 Interdepartmental Research Center "E. Piaggio", University of Pisa
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

classdef Limb < handle

    properties
        name;    % limb name
        manuf;   % limb manufacturer
        comment; % comment
    end
    properties (Dependent)
        P;       % origins of joint coordinates
        Z;       % joint axes
        n;       % limb's dofs (commodity parameter)
        joints;  % vector of joint types: 0 for rotational, 1 for prismatic joints
        tool;    % commodity alias for g_st0
        config;  % human-friendly representation of joint types
        g_st0;   % trasformation matrix from base to end-effector frame
    end
    properties (Access = protected)
        descr    % internal kinematic description of Limb
    end

    methods

        function obj = Limb(varargin)
            %
            % SYNOPSYS
            % l2 = Limb(l1)
            % l1 = Limb(Z, P, g_st0, 'PropertyName', PropertyValue, ...)
            % l1 = Limb('screw', Z, P, g_st0, 'PropertyName', PropertyValue, ...)
            % l1 = Limb('dh', 'PropertyName', PropertyValue, ...)
            %

            % default props
            obj.name = 'noname';
            obj.manuf = '';
            obj.comment = '';

            % process required user-providen options
            if nargin < 1,
                error('MATLAB:narginchk:notEnoughInputs', 'Not enough input arguments.');
            end
            arg1 = varargin{1};
            oplLimb = {};
            if isa(arg1, 'Limb'),
                [nr, nc] = size(arg1);
                lLength = nr*nc;
                if lLength == 1,
                    obj.name = arg1.name;
                    obj.manuf = arg1.manuf;
                    obj.comment = arg1.comment;
                    obj.descr = arg1.descr.copy;
                elseif any([nr,nc] == 1),
                    % copy an array
                    for i = 1:lLength,
                        obj(i) = Limb(arg1(i)); %#ok<AGROW>
                    end
                    if nr > 1, obj = obj'; end
                else
                    error('TRG:Limb:NonConsistentValue', 'Wrong input argument: expected an array of Limbs (nx1), not a matrix of Limbs.');
                end
            else
                % filter Limb's properties (name, etc.) from
                % description-related properties (if any)
                [oplLimb, oplDescription] = srv_filter_options_from_list({'name', 'manuf', 'comment'}, 3, varargin{:});
                if isnumeric(arg1),
                    obj.descr = LimbDescriptionFactory.create('screw', oplDescription{:});
                elseif ischar(arg1),
                    if size(arg1, 1) > 1,
                        error('TRG:Limb:NonConsistentValue', 'Wrong input argument: expected a string, not an array of strings.');
                    end
                    if any(strcmp(arg1, {'name', 'manuf', 'comment'})),
                        error('TRG:Limb:NonConsistentValue', 'Wrong input argument: trailing property cannot be ''name'', ''manuf'' or ''comment''.');
                    end
                    obj.descr = LimbDescriptionFactory.create(arg1, oplDescription{2:end});
                else
                    error('TRG:Limb:UnknownInput', 'Wrong input argument, expected a LimbPoE or numeric matrices.');
                end
            end

            % process additional user-providen options
            lOplLimb = length(oplLimb);
            if lOplLimb > 0,
                i = 1;
                while i < lOplLimb,
                    processedProp = oplLimb{i};
                    switch processedProp,
                        case {'name', 'manuf', 'comment'},
                            currentProp = oplLimb{i};
                            i = i + 1;
                            if ~ischar(oplLimb{i}),
                                error('TRG:Limb:NonConsistentValue', 'Wrong value specified for ''%s'', expected a string.', currentProp);
                            end
                            if size(oplLimb{i}, 1) > 1,
                                error('TRG:Limb:NonConsistentValue', 'Wrong value specified for ''%s'': expected a string, not an array of strings.', currentProp);
                            end
                            eval(['obj.', currentProp, ' = oplLimb{i};']);
                        otherwise,
                            error('TRG:Limb:UnknownProperty', 'Unknown property ''%s''.', processedProp);
                    end
                    i = i + 1;
                end
            end
        end

        function obj2 = copy(obj)
            %Limb.copy Clone a Limb object
            %
            % l2 = l1.copy() is a deepcopy of the object l1.
            obj2 = Limb(obj);
        end

        % set/get methods

        function set.name(obj, aString)
            if ~ischar(aString),
                error('TRG:setName:NonConsistentValue', 'Wrong value specified for ''name'', expected a string.');
            end
            if size(aString, 1) > 1,
                error('TRG:setName:NonConsistentValue', 'Wrong value specified for ''name'': expected a string, not an array of strings.');
            end
            obj.name = aString;
        end

        function set.manuf(obj, aString)
            if ~ischar(aString),
                error('TRG:setManuf:NonConsistentValue', 'Wrong value specified for ''manuf'', expected a string.');
            end
            if size(aString, 1) > 1,
                error('TRG:setManuf:NonConsistentValue', 'Wrong value specified for ''manuf'': expected a string, not an array of strings.');
            end
            obj.manuf = aString;
        end

        function set.comment(obj, aString)
            if ~ischar(aString),
                error('TRG:setComment:NonConsistentValue', 'Wrong value specified for ''comment'', expected a string.');
            end
            if size(aString, 1) > 1,
                error('TRG:setComment:NonConsistentValue', 'Wrong value specified for ''comment'': expected a string, not an array of strings.');
            end
            obj.comment = aString;
        end

        % dependent properties

        function P = get.P(obj)
            P = obj.descr.P;
        end

        function Z = get.Z(obj)
            Z = obj.descr.Z;
        end

        function n = get.n(obj)
            n = obj.descr.n;
        end

        function joints = get.joints(obj)
            joints = obj.descr.joints;
        end

        function set.g_st0(obj, transform)
            obj.descr.g_st0(transform);
        end

        function transform = get.g_st0(obj)
            transform = obj.descr.g_st0;
        end
        
        function set.tool(obj, transform)
            obj.descr.g_st0 = transform;
        end

        function transform = get.tool(obj)
            transform = obj.descr.g_st0;
        end

        function repr = get.config(obj)
            sJointTypes = ['R','P'];
            repr = sJointTypes((obj.joints)+1);
        end

    end

    methods (Access = protected)

        r = basePlot(limb, q, PlotOpt);

    end

    methods (Access = protected, Static)

        [D_J_s_k] = derivJacob0Kth(J_s, k);

    end

end