% GraspAnalysisSolver.COORDINATEPERTURBS. Compute the Columns that are Missing
% to Complete a Basis for the Subspace of External Forces and Displacements
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

function StructOut = coordinatePerturbs( grsp_poe )

    % early return
    StructOut = [];

    % external perturbs
    StructExt = grsp_poe.nullSpaceDecomp({},{'d_we'});
    if isempty(StructExt),
        return;
    end
    ext_c = struct2cell(StructExt);
    ext_m = cell2mat(ext_c);

    % external structural forces
    extStr_m = grsp_poe.methodOutStruct2Mat(grsp_poe.extStructForces);

    % find which columns in ext_m are linearly independent from the columns
    % of extStr_m
    co_m = llcs_vspacecomp_recovery(extStr_m, ext_m);

    % convert co_m to StructOut
    StructOut = grsp_poe.methodOutMat2Struct(co_m);
end