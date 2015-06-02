% LLC_HMANIP. Low-Level Computational Routine that Computes the Manipolability Subspaces for Rigid Grasps Without
% Synergies
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

 function [gOutRM] = llc_hmanip(g, ja, jp, tol)

% NW, NA
nw = size(g,1); na = size(ja,2); np = size(jp,2);
q = [eye(nw),      zeros(nw,na), -g;...
     zeros(na,nw), eye(na),      ja';...
     zeros(np,nw), zeros(np,na), jp'];
b = llcs_kernel(q); bu = b(1:nw+na, :); bl = b(nw+na+1:end, :);

% decomp: GMS, GMW, GMT, GMH, NS, NC, NH
[gms, gmw, gmt, gmh, e] = llcs_qr_decomp(bu, nw, tol);
ns = size(gms,2); nc = size(gmw,2); nh = size(gmh,2);

% arrange bl accordingly
bl = bl*e;

% extract gm from bl: GMFS, GMF, GMFH
gmfs = bl(:, 1:ns);
gmf = bl(:, ns+1:ns+nc);
gmfh = bl(:, end-nh+1:end);

% prepare the output
gOutRM = GrOutRigidManipulability(...
                'nw', nw, ...
                'na', na, ...
                'nh', nh, ...
                'nc', nc, ...
                'ns', ns, ...
                'gmh', gmh, ...
                'gmfh', gmfh, ...
                'gmw', gmw, ...
                'gmt', gmt, ...
                'gmf', gmf, ...
                'gms', gms, ...
                'gmfs', gmfs);