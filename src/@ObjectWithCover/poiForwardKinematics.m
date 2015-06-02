%ObjectWithCover.POIFORWARDKINEMATICS
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
%       - err check
%   Modified on September 2013
%       - reducedProb

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

function T = poiForwardKinematics(cvr, u)

    % checking rhs args
    narginchk(2,2);

    % check if cvr is a ObjectWithCover
    if ~isa(cvr, 'ObjectWithCover'),
        error('TRG:poiForwardKinematics:NonConsistentValue', ...
            'Wrong input argument, expected a ObjectWithCover object.');
    end

    % check conf
    if ~isa(u, 'numeric'),
        error('TRG:poiForwardKinematics:NonConsistentValue', ...
            'Wrong input argument, expected an object configuration vector (u).');
    end
    u = real(u(:));
    if numel(u) ~= cvr.pReducedProb.dim,
        error('TRG:poiForwardKinematics:NonConsistentValue', ...
            'Wrong object config, expected %d elements in place of %d.', ...
                cvr.pReducedProb.dim, numel(u));
    end

    % save a copy of internals for referencing them w/o func call
    ops = cvr.POIStructure;
    objModel = cvr.object;

    % switch to 3d comput. if the problem is 2d
    if cvr.isProblem2d,
        u = cvr.pReducedProb.restoreConfTo3d(u);
    end

    % compute object's fkine
    objFkineTr = objModel.fkine(u);

    % for each POI ...
    %
    % compute the forward kinematics
    T = cellfun(@(x) srv_compute_poi_fkine(x, objFkineTr), ...
                            {ops.pois.relativeTransform}', ...
                                'UniformOutput', false);

    % reduce to planar case (if needed)
    if cvr.isProblem2d,
        T = cellfun(@(x) cvr.pReducedProb.reduceHomogTransformTo2d(x), ...
                            T, ...
                                'UniformOutput', false);
    end

end

function T = srv_compute_poi_fkine(poiT, objFkineTr)
    T = objFkineTr*poiT;
end
