% LLCS_QR_DECOMP. Low-Level Service Computational Routine for QR Factorization
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

function [c11, c12, c22, c23, e] = llcs_qr_decomp(x, n, tol)

% upper section
xu = x(1:n,:);

% column space compression: 1st step
[p, r] = size(xu);
[m1, r1] = llcs_qr_colcomp(xu, tol);
m1 = m1(:, end:-1:1);

% perform an early return when n == 0
if n == 0,
    c11 = [];
    [xr, xc] = size(x);
    c12 = zeros(n, xc);
    c22 = x;
    s_ = zeros(xr, size(m1,2));
    c23 = s_(p+1:end, r1+1:end);
    e = eye(xc);
    return;
end

% continue when n != 0
s = x*m1;

% s has now the following structure
% [s11,  0 ]
% [s21, c23]
% where s11 has r1 columns and c23 has r-r1 columns
s11 = s(1:p, 1:r1);
s21 = s(p+1:end, 1:r1);
c23 = s(p+1:end, r1+1:end);

% column space compression: 2nd step, reduce s21
[m2, r2] = llcs_qr_colcomp(s21, tol);
s21 = s21*m2;

% s21 has now the following structure
% [0, c22]
% where c22 has r2 columns, so 0 has r1-r2 columns
c22 = s21(:, end-r2+1:end);

% arrange s11 accordingly
s11 = s11*m2;
%
% s11 has now the following structure
% [c11, c12]
c11 = s11(:, 1:end-r2);
c12 = s11(:, end-r2+1:end);

e = m1*[m2, zeros(r1,r-r1); zeros(r-r1,r1), eye(r-r1)];