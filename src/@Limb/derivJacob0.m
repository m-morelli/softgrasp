%Limb.DERIVJACOB0. Evaluates the derivative of Limb spatial Jacobian
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

function [D_J_s, D_J_s_cell] = derivJacob0(limb, q)

    % check rhs
    narginchk(2,2);

    % Limb and q are checked by jacob0()
    % nop.

    % spatial jacobian
    J_s = limb.jacob0(q);   % also check limb and q
    n = limb.n;

    % compute single contributions & collect them as cell array
    D_J_s_cell = cell(1,n);
    for k = 1:n,
        D_J_s_cell{k} = Limb.derivJacob0Kth(J_s,k);
    end

    % transform to a matrix
    D_J_s = cell2mat(D_J_s_cell);

end