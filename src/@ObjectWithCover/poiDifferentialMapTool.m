%ObjectWithCover.POIDIFFERENTIALMAPTOOL Transposed Complete Grasp Map in Body Coordinates
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

function gt = poiDifferentialMapTool(cvr)

    % check the number of inputs
    narginchk(1,1);

    % check if cvr is a ObjectWithCover
    if ~isa(cvr, 'ObjectWithCover'),
        error('TRG:poiDifferentialMapTool:UnknownInput', ...
            'Wrong input argument, expected a ObjectWithCover object.');
    end

    % save a copy of internals for referencing them w/o func call
    ops = cvr.POIStructure;

    % for each POI ...
    %
    % compute the corresponding contribute to the complete grasp matrix
    gti = cellfun(@(x) srv_build_grasp_matrix(x), ...
                    {ops.pois.relativeTransform}, ...
                        'UniformOutput', false);

    % finally, put all the contributes together
    gt = vertcat(gti{:});

    % reduce to planar case (if needed)
    if cvr.isProblem2d,
        gt = cvr.pReducedProb.reduceDiffMotionsMapTo2d(gt);
        gt = (cvr.pReducedProb.reduceObjectComponentsTo2d(gt'))';
    end

end

function [gti] = srv_build_grasp_matrix(poiT)
    Roci = poiT(1:3,1:3);
    poci = poiT(1:3,4);
    g3d = [                                 Roci,     zeros(3,3); ...
           [        0, -poci(3),  poci(2);...
              poci(3),        0, -poci(1);...
             -poci(2),  poci(1),        0 ]*Roci,     Roci];
    gti = g3d';
end

%{
function [gti, idxV, idxW] = srv_reduce_gti_to_2d(gti, reducedProb)
    g3d = gti'; idxV = []; idxW = [];
    switch reducedProb.ontoPlane,
        case 'xy',
            idxV = 1:2;
            idxW = 3;
            gti = [g3d(1:2,[idxV,6]); g3d(end,[idxV,6])]';
        otherwise,
    end
%}