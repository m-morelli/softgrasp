%HandWithGlove.POIDIFFERENTIALMAPTool. Complete Hand Jacobian in Tool Coordinates
%   Description, in progress
%
%   Annotations, in progress
%
%   References, in progress
%
%   Matteo Morelli, I.R.C. "E. Piaggio", University of Pisa
%
%   July 2013
%   October 2013: added j_s_cell output (collection of jacobian contributes of each finger)

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

function [j, j_s_cell] = poiDifferentialMapTool(glv, q)

    % number of inputs must be == 2
    narginchk(2, 2);

    % do the job
    [j, j_s_cell] = poiDifferentialMap(glv, q, @jacobn);

    % perform planar simplifications?
    if glv.isProblem2d,
        rp = glv.pReducedProb;
        j = rp.reduceDiffMotionsMapTo2d(j);
        j_s_cell = cellfun(@(x) rp.reduceObjectComponentsTo2d(x), ...
                            j_s_cell, ...
                                'UniformOutput', false);
    end