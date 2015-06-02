% LLCS_VDVHV. Low-Level Service Computational Routine that Evaluates the Merit Function (V), its gradient (DV), and HV
% its Hessian matrix (HV) during GFO computations
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

function [v,dv,hv,s] = llcs_vdvhv(go, yo)

    % extract info from optim struct.
    %
    % grasp related
    E=go.E;
    Gr=go.Grk;
    Fe=go.Fe;
    minF=go.minF;
    maxF=go.maxF;
    cs = go.contactStruct;
    cidx = 1:cs.nContactModels;
    cidx(go.excludeContacts) = [];
    %
    % optim options.
    de=go.de;
    wf=go.wf;
    wm=go.wm;
    wM=go.wM;

    % compute tc
    GrFe = -Gr*Fe;
    tc = GrFe + E*yo;

    % convert all the quantities of interest from vectors/matrices to lists (2 lists, forces & moments)
    [tcf, tcm] = cs.isolateSingleContributionsFrom(tc);
    [M, N] = cs.isolateSingleContributionsFrom(E);

    % init the cost function & the 'trusted regions'
    s_ = ''''''; s = [];
    x_fr = 0;  dx_fr = 0;  hx_fr = 0;
    x_min = 0; dx_min = 0; hx_min = 0;
    x_max = 0; dx_max = 0; hx_max = 0;
    de2 = de * de; de3 = de * de2; de4 = de2 * de2;
    if de ~= 0, % parab.
        aa = 3/(2*de4); bb = 4/de3; cc = 3/de2;
    else
        aa=0; bb=0; cc=0;
    end

    % compute the cost function
    for i=cidx,
        % extract the i-th contact model
        cl = cs.contactModels(i);

        % forces
        if ~isempty(tcf{i}),
            fic = tcf{i};                    % i-th contact force
            fin = max(sqrt(fic'*fic),1e-9);  %  ''   norm
            fiv = fic/fin;                   %  ''   vector
        else
            fic = 0;
            fin = 0;
            fiv = 0;
        end
        Mi = M{i};                  % corresponding piece in E

        % moments
        if ~isempty(tcm{i}),        % TODO maybe we should distingue the contact type, e.g.:
                                    % soft finger
                                    %      min_ = max(tcm(i),1e-9); miv = sign(min_);
                                    % hard finger
                                    %      min_ = 0;                miv = 0;           Ni = 0;
            mic = tcm{i};                    % i-th contact moment
            min_ = max(sqrt(mic'*mic),1e-9); %  ''   norm
            miv = mic/min_;                  %  ''   vector
        else
            %mic = 0;
            min_ = 0;
            miv = 0;
        end
        Ni = N{i};                  % corresponding piece in E

        % put all the useful values in variables for reuse
        ai = cl.frictionCoefficients;
        bi = 1/ai(2); % TODO: XXX cambiare per tutti!
        ai = 1/sqrt(1 + ai(1)^2); % TODO: soft-finger, complete constr.
        Cni = cl.contactFrameObjectSide(1:3,3);

        % contrib of friction
        s_fr  = ai*fin + bi*min_ - Cni'*fic;
        ds_fr = ai*Mi'*fiv + bi*Ni'*miv - Mi'*Cni;
        hs_fr = (ai*Mi'*(eye(3) - fiv * fiv')*Mi) / fin;
        if s_fr >= - de,
            x_fr  = x_fr + aa*s_fr^2 + bb*s_fr + cc;
            dx_fr = dx_fr + (2*aa*s_fr + bb)*ds_fr;
            hx_fr = hx_fr + 2*aa*(ds_fr*ds_fr') + (2*aa*s_fr + bb) * hs_fr;
            if (ai*fin - Cni'*fic) > 0,
                s = [s, 'T', num2str(i)]; %#ok<*AGROW> % Translational slippage
            elseif (ai*fin - Cni'*fic) == 0,        % TODO: ~exactly zero!
                s = [s, 't',num2str(i)]; % Translational slippage (marginal)
            elseif (ai*fin + bi*min_ - Cni'*fic) > 0,
                s = [s, 'S',num2str(i)]; % Rotational slippage
            elseif (ai*fin + bi*min_ - Cni'*fic) == 0,% TODO: ~exactly zero!
                s = [s, 's',num2str(i)]; % Rotational slippage (marginal)
            end
        else
            x_fr  = x_fr + 1/(2*s_fr^2);
            dx_fr = dx_fr - ds_fr / (s_fr^3);
            hx_fr = hx_fr + (3*(ds_fr*ds_fr')/s_fr - hs_fr)/(s_fr^3);
        end

        % contrib of min value of the normal force
        s_min = -Cni'*fic + minF;
        ds_min = - Mi'*Cni;
        hs_min =  0;
        if s_min >= - de,
            x_min  = x_min + aa*s_min^2 + bb*s_min + cc;
            dx_min = dx_min + (2*aa*s_min + bb)*ds_min;
            hx_min = hx_min + 2*aa*(ds_min*ds_min') + (2*aa*s_min + bb) * hs_min;
            if s_min > 0,
                s = [s, 'M',num2str(i)]; % Below Minimum
            elseif s_min == 0,                      % TODO: ~exactly zero!
                s = [s, 'm',num2str(i)]; % Marginal Minimum
            end
        else
            x_min  = x_min + 1/(2*s_min^2);
            dx_min = dx_min - ds_min / (s_min^3);
            hx_min = hx_min + (3*(ds_min*ds_min')/s_min - hs_min)/(s_min^3);
        end

        % contrib of max intensity of the contact force
        s_max  = fin - maxF;
        ds_max = Mi'*fiv;
        hs_max = (Mi'*(eye(3) - fiv * fiv')*Mi) / fin;
        if s_max >= - de,
            x_max  = x_max + aa*s_max^2 + bb*s_max + cc;
            dx_max = dx_max + (2*aa*s_max + bb)*ds_max;
            hx_max = hx_max + 2*aa*(ds_max*ds_max') + (2*aa*s_max + bb) * hs_max;
            if s_max > 0,
                s = [s, 'X',num2str(i)]; % Above Maximum
            elseif s_max == 0,                      % TODO: ~exactly zero!
                s = [s, 'x',num2str(i)]; % Marginal Maximum
            end
        else
            x_max  = x_max + 1/(2*s_max^2);
            dx_max = dx_max - ds_max / (s_max^3);
            hx_max = hx_max + (3*(ds_max*ds_max')/s_max - hs_max)/(s_max^3);
        end

    end

    if isempty(s), s = s_; end
    v  = wm*x_min  + wM*x_max  + wf*x_fr;
    dv = wm*dx_min + wM*dx_max + wf*dx_fr;
    hv = wm*hx_min + wM*hx_max + wf*hx_fr;