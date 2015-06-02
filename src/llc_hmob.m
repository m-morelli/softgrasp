% LLC_HMOB. Low-Level Computational Routine that Computes the Mobility Subspaces for Rigid Grasps Without Synergies
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

function [gOutRM] = llc_hmob(g, ja, jp, tol)

% NU, NA, NP
na = size(ja,2); np = size(jp,2); nu = size(g,1);
q = [ja, jp, -g'];
b = llcs_kernel(q);

% 1st decomp: GMAR, GMAC, NR
[gmar, gmac, gmpoc, gmi] = llcs_qr_decomp(b, na, tol);
nr = size(gmar,2);

% 2nd decomp (on gmi): GMPR, GMPOI, GMOPI, GMOI, NI
[gmpr, gmpoi, gmopi, gmoi] = llcs_qr_decomp(gmi, np, tol);
ni = [size(gmpr,2), size(gmpoi,2), size(gmoi,2)];

% 3rd decomp (on gmpoc): GMPAC, GMPOAC, GMOPAC, GMOAC, NC
[gmpac, gmpoac, gmopac, gmoac, e] = llcs_qr_decomp(gmpoc, np, tol);
%
% arrange GMAC accordingly
gmac = gmac*e;
nc = [size(gmpac,2), size(gmpoac,2), size(gmoac,2)];

% prepare the output
gOutRM = GrOutRigidMobility(...
                'nu', nu, ...
                'na', na, ...
                'np', np, ...
                'nr', nr, ...
                'nc', nc, ...
                'ni', ni, ...
                'gmar', gmar, ...
                'gmac', gmac, ...
                'gmpac', gmpac, ...
                'gmpoac', gmpoac, ...
                'gmopac', gmopac, ...
                'gmoac', gmoac, ...
                'gmpr', gmpr, ...
                'gmpoi', gmpoi, ...
                'gmopi', gmopi, ...
                'gmoi', gmoi);