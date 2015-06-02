% LLCS_QR_COLCOMP. Low-Level Service Computational Routine for Column Compression
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

function [w, rk] = llcs_qr_colcomp(a, tol)

if nargin < 1,
    error('insufficient number of input arguments');
end
w = []; rk = 0;
[ma, na]=size(a);
if nargin < 2,
    tol = sqrt(eps)*norm(a,1)*max(ma,na); 
end
if ~isempty(a),
    if norm(a,1) < sqrt(eps)/10,
        rk = 0;
        w = eye(na);
    else
        [q, r, e] = qr(a');
        rk = rank(diag(diag(r)), tol);
        w = q(:, na:-1:1);
    end
end    