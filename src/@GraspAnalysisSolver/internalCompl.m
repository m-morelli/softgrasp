% GraspAnalysisSolver.INTERNALCOMPL. Complete the Space of Internal Forces
%   Description, in progress
%
%   Annotations, in progress
%
%   References, in progress
%
%   Matteo Morelli, I.R.C. "E. Piaggio", University of Pisa
%   Edoardo Farnioli, I.R.C. "E. Piaggio", University of Pisa
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

function StructOut = internalCompl( grsp_poe )

    % early return
    StructOut = [];

    % internal perturbs
    StructInt = grsp_poe.nullSpaceDecomp({'d_we'},{});
    if isempty(StructInt),
        return;
    end
    int_c = struct2cell(StructInt);
    int_m = cell2mat(int_c);

    % pure squeeze (2)
    ps_m = grsp_poe.methodOutStruct2Mat(grsp_poe.pureSqueeze);

    % spurious squeeze (3)
    ss_m = grsp_poe.methodOutStruct2Mat(grsp_poe.spuriousSqueeze);

    % kinematic displacements (4)
    kd_m = grsp_poe.methodOutStruct2Mat(grsp_poe.kineGraspDispl);

    % redundant motions (5)
    rm_m = grsp_poe.methodOutStruct2Mat(grsp_poe.redundantMotions);

    % find which columns in int_m are linearly independent from the columns
    % of [(2), (3), (4), (5)]
    co_m = llcs_vspacecomp_recovery([ps_m, ss_m, kd_m, rm_m], int_m, 1e-8);

    % convert co_m to StructOut
    StructOut = grsp_poe.methodOutMat2Struct(co_m);
end