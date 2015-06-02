% LLCS_VDVHVSS. Low-Level Service Computational Routine that Performs the Step Optimization during GFO computations
%   Description, in progress
%
%   Annotations, in progress
%
%   References, in progress
%
%   Matteo Morelli, I.R.C. "E. Piaggio", University of Pisa
%
%   May 2012

% Copyright (C) 2012 Interdepartmental Research Center "E. Piaggio", University of Pisa
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

function [la, jj] = llcs_vdvhvss(go, y_k, d_k, flag)

    la = []; jj = [];
    rhs = nargin;
    if rhs < 4,
        [la, jj] = llcs_vdvhvss_gss(go, y_k, d_k);
    else
        switch flag,
            case 'gss',
                [la, jj] = llcs_vdvhvss_gss(go, y_k, d_k);
            case 'bis',
                [la, jj] = llcs_vdvhvss_bis(go, y_k, d_k);
            otherwise,
        end
    end

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
        warning('GRASP:classProps:NonConvergentSolution', ...
            ['golden section search alg. stopped after 100 steps with |la_-la| = ', num2str(abs(la_ - la))]);
    end

function [lx_, jj] = llcs_vdvhvss_bis(go, y_k, d_k)

    % finding step size:
    max_stepn = 10000; qqmax=1e13;
    tol = 1e-6;
    jj = 0; qq = 0;
    ll_ = 0;     [~,dv_l,~,~] = llcs_vdvhv(go, y_k + ll_*d_k); tp_ll  = dv_l'*d_k;
    lu_ = 1e-20; [~,dv_u,~,~] = llcs_vdvhv(go, y_k + lu_*d_k); tp_lu  = dv_u'*d_k;
    if tp_ll > 0, error('GRASP:classProps:WTF','argh! you shouldn''t be here...'); end;
    while tp_lu<=0 && qq<qqmax, ll_=lu_; lu_=2*lu_; [~,dv_u,~,~]=llcs_vdvhv(go, y_k+lu_*d_k); tp_lu=dv_u'*d_k; qq=qq+1; end;
    if qq == qqmax, error('GRASP:classProps:NonConvergentSolution','cannot find a value for lu_ such that tp_lu is > 0'); end
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
        warning('GRASP:classProps:NonConvergentSolution', ...
            ['bisection alg. stopped after 1e4 steps with lx_ = ' + sci2exp(lx_) + ' and |tp_lx| = ', num2str(abs(tp_lx))]);
    end
