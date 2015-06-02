% GraspForceOptimizer.LLCS_VDVHVSS_GSS. Perform the Step Optimization According to the Golden Section Search (GSS) Algorithm
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

function [la, jj] = llcs_vdvhvss_gss(go, y_k, d_k)

    % finding step size:
    max_stepn = 100;
    tol = 1e-6;
    c = (3-sqrt(5))/2;
    jj = 0;
    la  = 0;    [v] = llcs_vdvhv(go, y_k + la*d_k); v_la  = v;
    la_ = 2;    [v] = llcs_vdvhv(go, y_k + la_*d_k); v_la_ = v;
    while(v_la_-v_la<=0),
        la_=2*la_;  [v] = llcs_vdvhv(go, y_k + la_*d_k); v_la_ = v;
    end

    while (abs(la_ - la) > tol) && (jj < max_stepn),
        lb  = la  + c*(la_ - la);   [v] = llcs_vdvhv(go, y_k + lb*d_k);    v_lb  = v;
        lb_ = la_ - c*(la_ - la);   [v] = llcs_vdvhv(go, y_k + lb_*d_k);   v_lb_ = v;

        if abs(v_lb - v_lb_) < tol,

            la  = lb;  v_la  = v_lb;
            la_ = lb_; v_la_ = v_lb_;

        elseif  v_lb  <  v_lb_,

            if  v_la  <= v_lb,
                la_ = lb;  v_la_ = v_lb;
            else
                la_ = lb_; v_la_ = v_lb_;
            end

        else

            if  v_la_ <= v_lb_,
                la = lb_;  v_la  = v_lb_;
            else
                la = lb;   v_la  = v_lb;
            end

        end
        jj = jj + 1;
    end

    if jj == max_stepn,
        warning('TRG:optimize:NonConvergentSolution', ...
                    'The Golden Section Search alg. stopped after 100 steps with |la_-la| = %f.', abs(la_ - la));
    end

end