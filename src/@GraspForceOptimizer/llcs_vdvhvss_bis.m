% GraspForceOptimizer.LLCS_VDVHVSS_BIS. Perform the Step Optimization According to the Bisection (BIS) Algorithm
%   Description, in progress
%
%   Annotations, in progress
%
%   References, in progress
%
%   Matteo Morelli, I.R.C. "E. Piaggio", University of Pisa
%
%   May 2012
%   Modified on October 2013:
%       - code aligned with the new class organization

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

function [lx_, jj] = llcs_vdvhvss_bis(go, y_k, d_k)

    % finding step size:
    max_stepn = 10000; qqmax=1e13;
    tol = 1e-6;
    jj = 0; qq = 0;
    ll_ = 0;     [~,dv_l,~,~] = llcs_vdvhv(go, y_k + ll_*d_k); tp_ll  = dv_l'*d_k;
    lu_ = 1e-20; [~,dv_u,~,~] = llcs_vdvhv(go, y_k + lu_*d_k); tp_lu  = dv_u'*d_k;
    if tp_ll > 0, error('TRG:optimize:WTF','Argh! You shouldn''t be here...'); end;
    while tp_lu<=0 && qq<qqmax, ll_=lu_; lu_=2*lu_; [~,dv_u,~,~]=llcs_vdvhv(go, y_k+lu_*d_k); tp_lu=dv_u'*d_k; qq=qq+1; end;
    if qq == qqmax, error('TRG:optimize:NonConvergentSolution','Cannot find a value for lu_ such that tp_lu is > 0.'); end
    lx_ = (lu_ + ll_)/2;
    [~,dv_x,~,~] = llcs_vdvhv(go, y_k + lx_*d_k); tp_lx  = dv_x'*d_k;

    while (abs(tp_lx) > tol) && (jj < max_stepn),
        jj = jj + 1;
        if tp_lx > 0,
            lu_ = lx_;
        else
            ll_ = lx_;
        end
        lx_ = (lu_ + ll_)/2;
        [~,dv_x,~,~] = llcs_vdvhv(go, y_k + lx_*d_k); tp_lx  = dv_x'*d_k;
    end

    if jj == max_stepn,
        warning('TRG:optimize:NonConvergentSolution', ...
                    'The Bisection alg. stopped after 1e4 steps with lx_ = %f and |tp_lx| = %f.', lx_, abs(tp_lx));
    end

end