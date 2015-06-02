% GrOutCanonicalFGM.GRASPCOMPLIANCE. Compute the Grasp Compliance
%   Description, in progress
%
%   Annotations, in progress
%
%   References, in progress
%
%   Matteo Morelli, I.R.C. "E. Piaggio", University of Pisa
%   Edoardo Farnioli, I.R.C. "E. Piaggio", University of Pisa
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

function [mWu] = graspCompliance(goobj)

    % checks
    %
    % right number of rhs args?
    narginchk(1,1);

    % check if goobj is a GrOutCanonicalFGM
    if ~isa(goobj, 'GrOutCanonicalFGM'),
        error('TRG:controllableInternalForces:NonConsistentValue', ...
            'Wrong input argument, expected a GrOutCanonicalFGM object.');
    end

    % extract fundamental info from the FGMstruct
    mWu = -goobj.cFGMstruct.FGM( goobj.u_block_idx, goobj.d_we_idx );
end