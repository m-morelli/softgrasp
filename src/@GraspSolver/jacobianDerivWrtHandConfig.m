%GraspSolver.JACOBIANDERIVWRTHANDCONFIG. Derivative of the Jacobian Matrix with Respect to the Hand Configuration
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

function Q = jacobianDerivWrtHandConfig(grsp, q, u, fc)

    % checking rhs args
    %
    % right number of rhs args?
    narginchk(4,4);
    % check if grsp is a Grasp
    if ~isa(grsp, 'GraspSolver'),
        error('TRG:jacobianDerivWrtHandConfig:NonConsistentValue', ...
            'Wrong input argument, expected a GraspSolver object.');
    end
    % check q
    if ~isa(q, 'numeric'),
        error('TRG:jacobianDerivWrtHandConfig:NonConsistentValue', ...
            'Wrong input argument, expected a configuration vector (q).');
    end
    q = real(q(:));
    % check (eventually adapt) conf
    if ~isa(u, 'numeric'),
        error('TRG:jacobianDerivWrtHandConfig:NonConsistentValue', 'Wrong input argument, expected an object configuration vector (u).');
    end
    u = real(u(:));
    reducedProb = grsp.pReducedProb;
    if numel(u) ~= reducedProb.dim,
        error('TRG:jacobianDerivWrtHandConfig:NonConsistentValue', ...
            'Wrong object config, expected %d elements in place of %d.', ...
                reducedProb.dim, numel(u));
    end
    if reducedProb.isSet,
        u = reducedProb.restoreConfTo3d(u);
    end
    % check fc
    if ~isa(fc, 'numeric'),
        error('TRG:jacobianDerivWrtHandConfig:NonConsistentValue', 'Wrong input argument, expected a vector of contact preload (fc).');
    end
    fc = real(fc(:));

    % ctc models as cell array
    cm_cell = num2cell(grsp.contactStructure.contactModels);

    % derivative of the spatial Jacobian (complete cell-array format)
    D_Jtilde_s = grsp.handWithGlove.derivOfPoiDifferentialMapBase(q);

    % Ad_g_ba
    Ad_g_ba = llcs_iad2(grsp.objectWithCover.object.fkine(u));

    % force/moment components as cell arrays
    [ff_cell, mm_cell] = grsp.contactStructure.isolateSingleContributionsFrom(fc,reducedProb);

    % switch to 3d force/moment components if the problem is 2d
    [ff_cell, mm_cell] = cellfun(@(x,y,z) srv_zero_pad_2d_force_moment_components(x.forceMomentTransmittedDofs, y, z), ...
                                    cm_cell, ...
                                        ff_cell, ...
                                            mm_cell, ...
                                                'UniformOutput', false);

    % for each contact model...
    %
    Q_cell = cellfun(@(w,x,y,z) srv_build_finger_contrib_to_deriv_of_hand_jac(w, Ad_g_ba'*llcs_iad2(x.contactFrameObjectSide)'*x.selectionMatrix'*[y;z]), ...
                            D_Jtilde_s, ...
                                cm_cell, ...
                                    ff_cell, ...
                                        mm_cell, ...
                                            'UniformOutput', false);

    Q = blkdiag(Q_cell{:});
end

%
% service routines for the computation of derivative of complete hand Jacobian 
%
function Q_i = srv_build_finger_contrib_to_deriv_of_hand_jac(D_Ji_cell, Tx)

    Q_i = cellfun(@(x) x'*Tx, ...
                        D_Ji_cell, ...
                            'UniformOutput', false);
    Q_i = cell2mat(Q_i);

end

function [f_p, m_p] = srv_zero_pad_2d_force_moment_components(fmd, f, m)

    f_p = [               f; ...
            zeros( fmd(1)-numel(f), 1 )];

    m_p = [               m; ...
            zeros( fmd(2)-numel(m), 1 )];

end