%HandWithGlove.POIDIFFERENTIALMAP. Core Function for Complete Hand Jacobian Computation
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
%   October 2013: added j_s_cell output (collection of jacobian contributes of each finger)

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

function [j, j_serial_cell] = poiDifferentialMap(glv, q, jCbk)

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

    % check the callback
    if ~isa(jCbk, 'function_handle'),
        error('TRG:poiDifferentialMap:UnknownMethod', ...
            'Wrong input argument, expected the method for Jacobian computation (Limb object).');
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
    [j_cell, j_serial_cell] = cellfun(@(w,x,y,z) srv_build_hand_jacobian(w, x, y, z, jCbk), ...
                                    {hps.pois.relativeTransform}', ...
                                        cPOIBot, ...
                                            cRelevantJointConfig, ...
                                                cIninfluentJoints, ...
                                                    'UniformOutput', false);

    % put all the contributes together
    j = vertcat(j_cell{:});

    % this function performs the computations in 3d
    % since planar simplifications are performed by
    % the methods provided by the user interface, i.e,
    % poiDifferentialMapBase() and poiDifferentialMapTool()

%
% service routines for hand Jacobian computation
%
function [ja, jj] = srv_build_hand_jacobian(poiT, poiBot, qBot, inJ, jCbk)
    % compute the Jacobian contribution for the general (spatial) case
    zz1 = zeros(6, inJ(1));     % all the joints before
    zz2 = zeros(6, inJ(2));     % non-affecting joints + all the joints after
    if ~isempty(poiBot),
        poiBot.tool = poiBot.tool*poiT;
        jj = jCbk(poiBot, qBot);        % serial jacob
    else
        jj = zeros(6,0);
    end    
    % assemble the contribution
    ja = [zz1, jj, zz2];