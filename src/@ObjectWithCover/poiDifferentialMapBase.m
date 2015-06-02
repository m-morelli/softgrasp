%ObjectWithCover.POIDIFFERENTIALMAPBASE Transposed Complete Grasp Map in World Coordinates
%   Description, in progress
%
%   Annotations, in progress
%
%   References, in progress
%
%   Matteo Morelli, I.R.C. "E. Piaggio", University of Pisa
%
%   May 2012
%   Modified on July 2013
%       - branched from poiDifferentialMap method

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

function gt = poiDifferentialMapBase(cvr, u)

    % number of inputs should have been already checked in the caller
    narginchk(2,2);

    % check u
    if ~isa(u, 'numeric'),
        error('TRG:poiDifferentialMapBase:UnknownProperty', ...
            'Wrong input argument, expected an object configuration vector (u).');
    end
    u = real(u(:));
    if numel(u) ~= cvr.pReducedProb.dim,
        error('TRG:poiDifferentialMapBase:NonConsistentValue', ...
            'Wrong object config, expected %d elements in place of %d.', ...
                cvr.pReducedProb.dim, numel(u));
    end

    % switch to 3d comput. if the problem is 2d
    problemWas2d = false;
    if cvr.isProblem2d,
        cvr.pReducedProb.isSet = false;
        problemWas2d = true;
        u = cvr.pReducedProb.restoreConfTo3d(u);
    end

    % compute the corresponding the complete grasp matrix in Body coords.
    gb = (cvr.poiDifferentialMapTool)';

    % convert to World coords.
    g_ab = cvr.object.fkine(u);
    gs = llcs_itad2(g_ab)*gb;

    % transpose the result (differential map is G^T)
    gt = gs';

    % reduce to planar case (if needed)
    if problemWas2d,
        cvr.pReducedProb.isSet = true;
        gt = cvr.pReducedProb.reduceDiffMotionsMapBaseTo2d(gt);
        gt = (cvr.pReducedProb.reduceObjectComponentsTo2d(gt'))';
    end