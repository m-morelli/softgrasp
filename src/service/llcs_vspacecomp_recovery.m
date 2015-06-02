% LLCS_VSPACECOMP. Low-Level Service Computational Routine (With Numerical Error Recovery)
% for the Computation of the Vector Space Complement
%   Description, in progress
%
%   Annotations, in progress
%
%   References, in progress
%
%   Matteo Morelli, I.R.C. "E. Piaggio", University of Pisa
%   Edoardo Farnioli, I.R.C. "E. Piaggio", University of Pisa
%
%   August 2013

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

function [c] = llcs_vspacecomp_recovery(a, b, tol)

    % early ret.
    c = [];

    % comput
    narginchk(2,3);
    if nargin == 2,
        tol = 1e-8;
    end
    rank_a = rank(a, tol);
    dim_a = size(a,2);
    rank_b = rank(b, tol);
    if rank_b < rank_a,
        error('TRG:llcs_vspacecomp_recovery:NonConsistentValue', ...
                'Unexpected rank values: rank(B) < rank(A).');
    end
    if rank_a == 0,
        c = b;
        return;
    end

    % find the independent columns
    num_indip = rank_b - rank_a;
    idx = dim_a + 1;
    o_idx = idx;
    tmp_a = a;
    tmp_b = b;
    z_star = 0;
    j_star = [];
    for i = 1:num_indip,
        for j = 1:size(tmp_b,2),
            x_star = pinv(tmp_a)*tmp_b(:,j);
            v_star = norm(tmp_a*x_star - tmp_b(:,j));
            if v_star > z_star,
                z_star = v_star;
                j_star = j;
            end
        end
        tmp_a(:,idx) = tmp_b(:,j_star);
        idx = idx + 1;
        tmp_b(:,j_star) = [];
        z_star = 0;
    end

    % assign to result
    c = tmp_a(:,o_idx:end);