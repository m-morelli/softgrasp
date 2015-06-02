%HandWithGlove.DERIVOFPOIDIFFERENTIALMAPBASE. Function for the Computation of the Derivative of the Spatial Hand Jacobian
%   Description, in progress
%
%   Annotations, in progress
%
%   References, in progress
%
%   Matteo Morelli, I.R.C. "E. Piaggio", University of Pisa
%
%   May 2012
%   July 2013: Update for use with LimbPoE objects

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

function dj = derivOfPoiDifferentialMapBase(glv, q)

    % number of inputs should have been already checked in the caller

    % check if glv is a HandWithGlove
    if ~isa(glv, 'HandWithGlove'),
        error('TRG:poiDifferentialMap:UnknownInput', ...
            'Wrong input argument, expected a HandWithGlove object.');
    end

    % check q
    if ~isa(q, 'numeric'),
        error('TRG:poiDifferentialMap:UnknownProperty', ...
            'Wrong input argument, expected a configuration vector (q).');
    end
    q = real(q(:));

    % save a copy of internals for referencing them w/o func call
    hnd = glv.hand;
    hps = glv.POIStructure;

    % check q (cont'd)
    if numel(q) ~= hnd.dofs,
        error('TRG:poiDifferentialMap:NonConsistentValue', ...
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

	% compute the corresponding contribute to the hand Jacobian
    [dj] = cellfun(@(w,x,y,z) srv_build_deriv_of_hand_jacobian(w, x, y), ...
                                {hps.pois.relativeTransform}', ...
                                    cPOIBot, ...
                                        cRelevantJointConfig, ...
                                                'UniformOutput', false);

    % reduce to planar case (if needed)
    if glv.isProblem2d,
        rp = glv.pReducedProb;
        dj = cellfun(@(x) srv_reduce_deriv_of_hand_jac(rp, x), ...
                            dj, ...
                                'UniformOutput', false);
    end
end

%
% service routines for hand Jacobian computation
%
function [Dj] = srv_build_deriv_of_hand_jacobian(poiT, poiBot, qBot)
    if ~isempty(poiBot),
        poiBot.tool = poiBot.tool*poiT;
        [~,Dj] = poiBot.derivJacob0(qBot);
    else
        Dj = zeros(6,0);
    end
end

function r_dj_k = srv_reduce_deriv_of_hand_jac(rp, dj_k)
    r_dj_k = cellfun(@(x) rp.reduceDiffMotionsMapTo2d(x), ...
                        dj_k, ...
                            'UniformOutput', false);
end