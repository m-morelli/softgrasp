%HandWithGlove.POIFORWARDKINEMATICS
%   Description, in progress
%
%   Annotations, in progress
%
%   References, in progress
%
%   Matteo Morelli, I.R.C. "E. Piaggio", University of Pisa
%
%   December 2012

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

function T = poiForwardKinematics(glv, q)

    % right number of rhs args?
    narginchk(2,2);

    % check if glv is a HandWithGlove
    if ~isa(glv, 'HandWithGlove'),
        error('TRG:poiForwardKinematics:NonConsistentValue', ...
            'Wrong input argument, expected a HandWithGlove object.');
    end

    % check conf
    if ~isa(q, 'numeric'),
        error('TRG:poiForwardKinematics:NonConsistentValue', ...
            'Wrong input argument, expected a configuration vector (q).');
    end
    q = real(q(:));
    % save a copy of internals for referencing them w/o func call
    hnd = glv.hand;
    hps = glv.POIStructure;
    %
    if numel(q) ~= hnd.dofs,
        error('TRG:poiForwardKinematics:NonConsistentValue', ...
            'Wrong joint config, expected %d elements in place of %d.', ...
                hnd.dofs, numel(q));
    end

    % for each POI ...
    %
    % determine how many joints do not affect the hand Jacobian computation
    cIninfluentJoints = cellfun(@(x) srv_identify_ininfluent_joint_set(x, hnd.dofsPerLimb), ...
                            hps.attachIndices, 'UniformOutput', false);

	% select the single joint set that is relevant for the hand Jacobian computation
    cRelevantJointConfig = cellfun(@(x,y) srv_collect_relevant_joint_set(q, x, y), ...
                            cIninfluentJoints, hps.attachIndices, 'UniformOutput', false);

    % build a special Limb object for hand Jacobian computation
    cPOIBot = cellfun(@(x) srv_limb_to_segment(x, hnd.limbs, hnd.palm), ...
                            hps.attachIndices, 'UniformOutput', false);

    % compute the forward kinematics
    T = cellfun(@(x,y,z) srv_compute_poi_fkine(x, y, z, hnd.palm), ...
                            {hps.pois.relativeTransform}', ...
                                cPOIBot, ...
                                    cRelevantJointConfig, ...
                                        'UniformOutput', false);

    % reduce to planar case (if needed)
    if glv.isProblem2d,
        T = cellfun(@(x) glv.pReducedProb.reduceHomogTransformTo2d(x), ...
                            T, ...
                                'UniformOutput', false);
    end
end

function T = srv_compute_poi_fkine(poiT, poiBot, qBot, palmTr)

    T = palmTr;
    if ~isempty(poiBot),
        T = T*poiBot.fkine(qBot);
    end
    T = T*poiT;
end