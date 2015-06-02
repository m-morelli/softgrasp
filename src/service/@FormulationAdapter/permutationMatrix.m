%FormulationAdapter.PERMUTATIONMATRIX. Permutation Matrix for Use With Adapter Class
%   Description, in progress
%
%   Annotations, in progress
%
%   References, in progress
%
%   Edoardo Farnioli, I.R.C. "E. Piaggio", University of Pisa
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

function S = permutationMatrix(n)

    % no error checking is performed
    % this routine is for service purposes only!
    % it is assumed the the caller knows what it is doing

    % permutation matrix
    i6n = eye(6*n);
    i6n_ = reshape(i6n, 6*n, 3, 2*n);
    S_ = squeeze(num2cell(i6n_,[1 2]));
    S = [horzcat(S_{1:2:2*n-1}), horzcat(S_{2:2:2*n})]';