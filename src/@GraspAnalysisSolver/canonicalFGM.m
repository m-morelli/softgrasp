% GraspAnalysisSolver.CANONICALFGM. Return the Fundamental Grasp Matrix in
% Canonical Form
%   Description, in progress
%
%   Annotations, in progress
%
%   References, in progress
%
%   Nicola Greco, I.R.C. "E. Piaggio", University of Pisa
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

function goCfgm = canonicalFGM(grsp)

    % check input arg.
    if ~isa(grsp, 'GraspAnalysisSolver'),
        error('TRG:canonicalFGM:NonConsistentValue', ...
            'Wrong input argument, expected a GraspAnalysisSolver object.');
    end

    % fast access to the FGMStruct
    A = grsp.FGMstruct;

    % no input args: only 'd_we' and the commanded displacement of the last level are allowed
    cfgm_c = {'d_we', A.conf{A.pointers(2)}};

    % check the dimensions
    dim_tot = grsp.fgmStructFindConf(cfgm_c{1}) + grsp.fgmStructFindConf(cfgm_c{2});
    [nr_fgm, nc_fgm] = size(A.FGM);
    dim_fgm = abs(nr_fgm-nc_fgm);
    if( dim_tot ~= dim_fgm )
        error('TRG:canonicalFGM:NonConsistentValue', ...
            'The inputs total dimension must be equal to the row-column difference of the FGM.');
    end

    % shift the configuration
    A = grsp.fgmStructShiftConf({cfgm_c{1},...
                                 cfgm_c{2}});

    % generate indices
    t = nr_fgm;
    I_idx = 1:t;
    Adi_idx = t+1:nc_fgm;

    % do actual computation!
    B = A.FGM(:,I_idx);
    if(rank(B) < t)
        error('TRG:canonicalFGM:NonConsistentValue', ...
            'The left block of the FGM is not full rank: inversion is impossible.');
    end
    A.FGM = [B\A.FGM(:,I_idx), B\A.FGM(:,Adi_idx)]; % cFGM

    % round the elements to the 14th decimal
    A.FGM = roundn(A.FGM, -14);

    % output data struct
    goCfgm = GrOutCanonicalFGM(...
            'cFGMstruct',   A,...
            'I_idx',        I_idx,...
            'Adi_idx',      Adi_idx,...
            'f_block_idx',  srv_build_index_vector('d_fc', A),...
            'u_block_idx',  srv_build_index_vector('d_u', A),...
            'd_we_idx',     t+1:t+A.dim(end-1),...
            'd_last_idx',   nc_fgm-A.dim(A.pointers(2))+1:nc_fgm);
end

%%%%%
function idx_vect = srv_build_index_vector(str, A)
    block_pos = find(strcmp(str, A.conf));
    block_idx_start_0 = sum(A.dim(1:block_pos-1));
    block_idx_finish = block_idx_start_0 + A.dim(block_pos);
    idx_vect = (block_idx_start_0+1):(block_idx_finish);
end