%FormulationAdapter.SELECTIONMATRIX. Selection Matrix (H) in World Coordinates
%   Description, in progress
%
%   Annotations, in progress
%
%   References, in progress
%
%   Matteo Morelli, I.R.C. "E. Piaggio", University of Pisa
%
%   May 2012
%   Modified on August 2013
%       - Moved to a method of the service class FormulationAdapter
%       - removed error checking

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

function H = selectionMatrix(contactModels, u, reducedProb)

    % no error checking is performed
    % this routine is for service purposes only!
    % it is assumed the the caller knows what it is doing

    % for each contact model ...
    hfs = []; hms = [];
    for i = 1:length(contactModels),
        hfs = blkdiag(hfs, contactModels(i).forceSelector(reducedProb));
        hms = blkdiag(hms, contactModels(i).momentSelectorBase(u, reducedProb));
    end

    % put all the contributes together
    H = blkdiag(hfs, hms);

    % and get rid of null rows
    H(~any(H,2), :) = [];