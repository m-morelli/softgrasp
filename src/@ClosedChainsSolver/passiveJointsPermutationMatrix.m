%ClosedChainsSolver.PASSIVEJOINTSPERMUTATIONMATRIX. Compute a Suitable Permutation Matrix that Reorders Joint Variables to Have
% Actuated Joints on top, and Unactuated Joints at Bottom
%   Description, in progress
%
%   Annotations, in progress
%
%   References, in progress
%
%   Matteo Morelli, I.R.C. "E. Piaggio", University of Pisa
%
%   June 2012
%   Modified on July 2013
%       - Moved (and adapted) from @Hand/ to @ClosedChainsSolver/

% Copyright (C) 2012, 2013 Interdepartmental Research Center "E. Piaggio", University of Pisa
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

function [P, qal] = passiveJointsPermutationMatrix(ccs)

    % checking rhs args
    narginchk(1,1);

    % check if hnd is a ClosedChainsSolver
    if ~isa(ccs, 'ClosedChainsSolver'),
        error('TRG:ClosedChainsSolver:NonConsistentValue', ...
            'Wrong input argument, expected a ClosedChainsSolver object');
    end

    % the hand
    hand = ccs.handWithGlove.hand;

    % compute the permutation matrix
    n = hand.dofs;
    qai = ccs.activeJoints;
    qal = length(qai);
    qpi = 1:n; qpi(qai) = [];
    qdel = 0:n:n*(n-1);
    st = zeros(n, n); st(qai + qdel(1:qal)) = 1; st(qpi + qdel(qal+1:end)) = 1;
    P = inv(st');