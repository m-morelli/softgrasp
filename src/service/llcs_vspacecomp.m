% LLCS_VSPACECOMP. Low-Level Service Computational Routine for the
% Computation of the Vector Space Complement
%   Description, in progress
%
%   Annotations, in progress
%
%   References, in progress
%
%   Matteo Morelli, I.R.C. "E. Piaggio", University of Pisa
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

function [c] = llcs_vspacecomp(a, b, tol)

    narginchk(2,3);
    if nargin == 2,
        tol = 1e-8;
    end
    rank_a = rank(a, tol);
    dim_a = size(a,2);
    idx = dim_a + 1;
    o_idx = idx;
    tmp_m = a;
    for i = 1:size(b,2),
        tmp_m(:,idx) = b(:,i);
        rank_tmp = rank(tmp_m, tol);
        if rank_tmp > rank_a,
            idx = idx + 1;
            rank_a = rank_tmp;
        end
    end
    c = tmp_m(:,o_idx:end);