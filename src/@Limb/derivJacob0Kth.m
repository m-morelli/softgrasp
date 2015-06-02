%Limb.DERIVJACOB0KTH. Evaluates the contribute of the k-th joint to the derivative of Limb spatial Jacobian
%   Description, in progress
%
%   Annotations, in progress
%
%   References, in progress
%
%   Nicola Greco, I.R.C. "E. Piaggio", University of Pisa
%   Matteo Morelli, I.R.C. "E. Piaggio", University of Pisa
%
%   September 2013

% Copyright (C) 2013 Interdepartmental Research Center "E. Piaggio", University of Pisa
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

function [D_J_s_k] = derivJacob0Kth(J_s, k)

    % internal use only, don't check J_s
    narginchk(2,2);

    % check the derivative index
    [v_dim, n] = size(J_s);
    if k < 0 || k > n,
        error('TRG:derivJacob0:NonConsistentValue', ...
            'Wrong derivative index, expected a number between 1 and %d.', ...
                n);
    end

    % compute and assembly single contributions
    pre_blk = zeros(v_dim,k);
    post_blk = zeros(v_dim,n-k);
    for j = (k+1):n,
        post_blk(:,j-k) = llcs_ad_lie(J_s(:,k))*J_s(:,j);
    end
    D_J_s_k = [pre_blk, post_blk];

end