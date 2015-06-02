%FormulationAdapter.JACOBIANMATRIXADAPTER. Jacobian Matrix Adapter Between Different Formulations of Grasp Analysis
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

function Jtx = jacobianMatrixAdapter(hwg, q, n)

    % no error checking is performed
    % this routine is for service purposes only!
    % it is assumed the the caller knows what it is doing

    % Adjoint (block-diag) matrix
    txi = cellfun(@(x) blkdiag(ext_rot(x),ext_rot(x))*llcs_iad2(x), ...
                        hwg.poiForwardKinematics(q), ...
                            'UniformOutput', false);
    Tx = blkdiag(txi{:});

    % permutation matrix
    S = FormulationAdapter.permutationMatrix(n);

    % Resulting adapter matrix
    Jtx = S*Tx;