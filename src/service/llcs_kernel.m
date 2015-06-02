% LLCS_KERNEL. Low-Level Service Computational Routine for Kernel Space Computation
%   Description, in progress
%
%   Annotations, in progress
%
%   References, in progress
%
%   Matteo Morelli, I.R.C. "E. Piaggio", University of Pisa
%
%   May 2012

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

function [q] = llcs_kernel(f)

%  [Q] = kernel(F)
%  Returns an orthogonal basis of the nullspace of F
%  The basis vectors are NOT normalized.
%  Q has as many columns as the nullity of F.

ft = f';
[ftr, ftc] = size(ft);

% N(f) = oc(R(f'))
ft = orth(ft);
[nr, nc] = size(ft);
ft = [ft, eye(ftr)];
q = orth(ft);
qc = size(q,2);
q = q(:, (nc+1):qc);