% GraspSolver.PLOT. Plot a Grasp
%   Description, in progress
%
%   Annotations, in progress
%
%   References, in progress
%
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

function plot(grsp, q, u, fc_)

    % rhs args + defaults
    narginchk(3,4);
    fc = [];

    % basic checks
    if ~isa(grsp, 'GraspSolver'),
        error('TRG:plot:NonConsistentValue', ...
            'Wrong input argument, expected a GraspSolver object.');
    end

    % shortcuts
    glv = grsp.handWithGlove;
    cvr = grsp.objectWithCover;
    cts = grsp.contactStructure;
    rp = grsp.pReducedProb;
    PlotOpt = grsp.PlotOpt;
    problemIs2d = grsp.isProblem2d;

    % basic checks (cont'd)
    if ~isa(q, 'numeric'),
        error('TRG:plot:NonConsistentValue', ...
            'Wrong input argument, expected a configuration vector (q).');
    end
    [tjPnts, nJtns] = size(q);
    if nJtns ~= glv.hand.dofs,
        error('TRG:plot:NonConsistentValue', ...
                'Wrong input argument, expected a configuration vector (q) with %d columns.',...
                    glv.hand.dofs);
    end
    if ~isa(u, 'numeric'),
        error('TRG:plot:NonConsistentValue', ...
            'Wrong input argument, expected an object configuration vector (u).');
    end
    [tuPnts, nUcomp] = size(u);
    if nUcomp ~= rp.dim,
        error('TRG:plot:NonConsistentValue', ...
            'Wrong object config, expected %d elements in place of %d.', ...
                rp.dim, nUcomp);
    end
    if tjPnts ~= tuPnts,
        error('TRG:plot:NonConsistentValue', ...
                'Wrong input argument, joint and object trajectories have different sizes (%d vs %d).',...
                    tjPnts, tuPnts);
    end
    if nargin > 3,
        if ~isa(fc_, 'numeric'),
            error('TRG:plot:NonConsistentValue', ...
                'Wrong input argument, expected a vector of contact forces (fc).');
        end
        if ~isempty(fc_),
            [fcf, ~] = cts.isolateSingleContributionsFrom(fc_, rp, true);
            if numel(fcf) ~= cts.nContactModels,
                error('TRG:plot:NonConsistentValue', ...
                    'Wrong input argument, expected %d contact forces instead of %d.', ...
                        cts.nContactModels, numel(fcf));
            end
            fcf = cellfun(@(f) srv_adjust_contact_force_vector(f, rp), fcf, 'UniformOutput', false);
            %{
            wr_f = find(cellfun(@(f) size(f,1) ~= rp.dimPos, fcf, 'UniformOutput', true));
            if ~isempty(wr_f),
                error('TRG:plot:NonConsistentValue', ...
                    'Wrong input argument, contact force(s): %s, have components which are not compatible with the current GrOpt2dProblem settings.', ...
                        num2str(wr_f));
            end
            %}
            if tjPnts ~= size(fc_,2),
                error('TRG:plot:NonConsistentValue', ...
                        'Wrong input argument, joint configurations (q) and contact forces (fc) have different sizes (%d vs %d).',...
                                tjPnts, size(fc_,2));
            end
            fc = fcf;
        end
    end

    % do plot!
    glv.plot(q(1,:), PlotOpt);
    if ~problemIs2d,
        u_3d = u;
    else
        u_3d = zeros(tuPnts, 6);
        u_3d(1,:) = rp.restoreConfTo3d(u(1,:))';
    end
    cvr.plot(u_3d(1,:), PlotOpt);
    T1 = cvr.poiForwardKinematics(u_3d(1,:));
    cts.plot(T1, PlotOpt, rp);
    if ~isempty(fc),
        Orig = cellfun(@(t1) t1(1:3,4), T1, 'UniformOutput', false);
        Dest = cellfun(@(f, t1) t1(1:3,1:3)*f(:,1), fc, T1, 'UniformOutput', false);
        cellfun(@(o,d) ext_arrow3(o, d, 'm', 2), Orig, Dest, 'UniformOutput', false);
    end

    % animate!
    if tjPnts > 1,
        for i = 2:tjPnts,
            clf;
            glv.plot(q(i,:), PlotOpt);
            if problemIs2d,
                u_3d(i,:) = rp.restoreConfTo3d(u(i,:))';
            end
            cvr.plot(u_3d(i,:), PlotOpt);
            Ti = cvr.poiForwardKinematics(u_3d(i,:));
            cts.plot(Ti, PlotOpt);
            if ~isempty(fc),
                Orig = cellfun(@(ti) ti(1:3,4), Ti, 'UniformOutput', false);
                Dest = cellfun(@(f, ti) ti(1:3,1:3)*f(:,i), fc, Ti, 'UniformOutput', false);
                cellfun(@(o,d) ext_arrow3(o, d, 'm', 2), Orig, Dest, 'UniformOutput', false);
            end
            pause(0.005);
        end
    end

    % TODO:
    % if tjPnts == 1 BUT size(fc,2) > 1 => multiple colors

    if(strcmp(PlotOpt.labels,'on'))
        F = [0 0 -2]';
        fh=text(F(1),F(2),F(3), grsp.name,'FontName','FixedWidth');
        set(fh,'fontWeight','bold');
    end

end

function fcf = srv_adjust_contact_force_vector(f, rp)

    [fr, fc] = size(f);
    if fr == 3,
        fcf = f;
    else
        fcf = zeros(3,fc);
        for j = 1:fc,
            fcf(:,j) = rp.restoreVectorTo3d(f(:,j));
        end
    end
    
end