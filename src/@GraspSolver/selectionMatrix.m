%GraspSolver.SELECTIONMATRIX. Selection Matrix in Object Coordinates
%   Description, in progress
%
%   Annotations, in progress
%
%   References, in progress
%
%   Matteo Morelli, I.R.C. "E. Piaggio", University of Pisa
%
%   May 2012
%   Modified on July 2013
%       - removed u as input arg according to force/moment selector
%       computations

% Copyright (C) 2012, 2013 Interdepartmental Research Center "E. Piaggio", University of Pisa
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

function h = selectionMatrix(grsp)

    % checking rhs args
    %
    % right number of rhs args?
    narginchk(1,1);
    % check if grsp is a Grasp
    if ~isa(grsp, 'GraspSolver'),
        error('TRG:selectionMatrix:NonConsistentValue', ...
            'Wrong input argument, expected a GraspSolver object.');
    end
    % reduced Prob?
    reducedProb = grsp.pReducedProb;
    % save a copy of internals for referencing them w/o func call
    cm = grsp.contactStructure.contactModels;

    % for each contact model...
    %
    % compute the corresponding contribute to the selection matrix
    hi = cellfun(@(x) x.selectionMatrix(reducedProb), ...
                        num2cell(cm), ...
                            'UniformOutput', false);

    % finally, put all the contributes together
    h = blkdiag(hi{:});