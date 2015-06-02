% SRV_FILTER_OPTIONS_FROM_LIST. Service Routine that Filters Specific Attributes from a Cell Array List of Options
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

function [oplSubClass, oplSuperClass] = srv_filter_options_from_list(strSubClassProps, nSubClassProps, varargin)
    %  initialize the option lists
    oplSubClass = {}; oplSuperClass = varargin;

    % extract SubClass-related props from varargin
    for i = 1:nSubClassProps,

        % identify the occurrencies of the i-th prop
        idxSubClassProp = find(strcmp(strSubClassProps(i), oplSuperClass));

        % select the final occurrency from varargin
        if ~isempty(idxSubClassProp),
            % filter duplicates, if any
            if length(idxSubClassProp) > 1,
                warning('TRG:srv_filter_options_from_list:MultipleDefs', 'Found multiple definitions for property ''%s'': using the last one.', strSubClassProps{i});
                idxSubClassProp = idxSubClassProp(end);
            end
            selectedOccur = oplSuperClass(idxSubClassProp:idxSubClassProp+1);
            oplSubClass(end+1:end+2) = selectedOccur;
            % finally remove the selected entry from varargin
            oplSuperClass(idxSubClassProp:idxSubClassProp+1) = [];
        end

    end
end