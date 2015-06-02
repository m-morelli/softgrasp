%Limb.PLOT. Perform 3D Graphical Animation of a Limb Model for a Given
%Joint-Set
%   Description, in progress
%
%   Annotations, in progress
%
%   References, in progress
%
%   Nicola Greco, I.R.C. "E. Piaggio", University of Pisa
%
%   March 2013

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

function retval = basePlot(limb, q, PlotOpt)

    % q must be a row vector. It is assumed as a row vector
    % No err check performed.

    % ret
    retval = [];

    % tmp vars
	P = limb.P;
    Z = limb.Z;
    EE = transl(limb.g_st0);
    joints = limb.descr.joints;
    n = limb.descr.n;
    homog_base2frameLINKS = zeros(4,4,n);
    homog_base2frameJOINTS = zeros(4,4,n);

    % base transformations for links and joints
	g0 = base_to_frame_serialchain(P, EE, n);
	g0JOINTS = base_to_frame_serialchainJOINTS(P, Z, n);

    % figure handling
    if isempty(findobj(gcf,'type','axes'))
    	ax = axes;
        grid on;
        axis equal;

        if isempty(PlotOpt.camView)
            view(3)
        else
            view(PlotOpt.camView);
        end

            if isempty(PlotOpt.Xlimit)
                set(ax,'XLimMode','auto');
            else
                set(ax,'xlim',PlotOpt.Xlimit);
            end
            if isempty(PlotOpt.Ylimit)
                set(ax,'YLimMode','auto');
            else
                set(ax,'ylim',PlotOpt.Ylimit);
            end
            if isempty(PlotOpt.Zlimit)
                set(ax,'ZLimMode','auto');
            else
                set(ax,'zlim',PlotOpt.Zlimit);
            end

            xlabel(PlotOpt.Xlabel);ylabel(PlotOpt.Ylabel);zlabel(PlotOpt.Zlabel);
            camlight left;
	else
            ax = gca;
    end

	for k = 1 : n   % for each joint belonging to the serial chain 

            % Calls the forward kinematics function. Each homog(:,:,i)
            % represents the homogeneous transformation from base to (i+1)th joint or link.

            homog_base2frameLINKS(:,:,k) = forward_kinematics_NOSYM(P, Z, g0(:,:,k), q(1:k), joints);

            if(limb.n == 1),
                homog_base2frameJOINTS(:,:,1) = rt2tr(srv_a2noa(Z(:,1)), P(:,1));
            else
                homog_base2frameJOINTS(:,:,k) = forward_kinematics_NOSYM(P, Z, g0JOINTS(:,:,k), q(1:k), joints);
            end

	end

	% plot core
	srv_plot_core(ax, homog_base2frameLINKS, homog_base2frameJOINTS, P(:,1), PlotOpt, joints);

end

function g0 = base_to_frame_serialchain(P, ee, num_el)

    for i = 1 : num_el    
        if(i == num_el)
            if(norm(ee-P(:,i)) == 0)
                g0(:,:,i) = rt2tr(eye(3),[0 0 0]);
            else
                rotazione = srv_a2noa(ee-P(:,i));
                g0(:,:,i) = rt2tr(rotazione,ee);
            end
        else
            next = i+1;
            if(norm(P(:,next)-P(:,i)) == 0)
                g0(:,:,i) = rt2tr(eye(3),P(:,i));
            else
                rotazione = srv_a2noa(P(:,next)-P(:,i));
                g0(:,:,i) = rt2tr(rotazione,P(:,next));
            end
        end
    end
end

function g0J = base_to_frame_serialchainJOINTS(P,Z,num_el)

    for i = 1 : num_el
        rotazione = srv_a2noa(Z(:,i));
        g0J(:,:,i) = rt2tr(rotazione,P(:,i));
    end

end

% forward_kinematics
%
% Evaluates Forward Kinematics for a serial chain; this function requires
% the user for the configuration in which will be calculated the
% kinematics.
%
%   Inputs:
%   P = Joint origins coordinates
%   Z = Joint axis
%   g_st0 = trasformation matrix from base to end-effector frame
%   q0 = Joint configuration in which will be evaluated the kinematics(degrees)
%   the 5-th optional input is a vector containing which are the prismatic joints
%
%   Outputs:
%   T = trasformation matrix from Base to End-Effector frame
%
%
%   Usage Example: T = forward_kinematics(P,Z,g_st0,[.1 .2 .4 .9],[1 4])
%
%   Means that we want to evaluate the kinematics for a 4-joint chain, of
%   which the 1st and the 4th are prismatic joints.
%

function T = forward_kinematics_NOSYM(P,Z,g_st0,q0,varargin)

    for i = 1 : size(Z,2)
        Z(:,i) = unit(Z(:,i));
    end
    qd = size(q0,2);  % takes the joint number from the joint axis
    joints = zeros(1,qd);    % joints vector: 0 for rotational, 1 for prismatic joints


    if(nargin == 5) % checks if there is an input containing the prismatic joints configuration
        vett = varargin{1};
        for i = 1 : size(vett,2)
            joints(i) = vett(i);
        end
    end

    for i = 1 : qd   % for all the joints

        if(joints(i) == 1) % prismatic
            xi_i = [Z(:,i); zeros(size(Z(:,i),1),1)];   % twist

        else % revolute
            xi_i = [-cross(Z(:,i),P(:,i)) ; Z(:,i)];    % twist
        end
        g_i = llcs_twistexp2(xi_i, q0(i));
        if(i == 1)
            T = g_i;
        else
            T = T*g_i;
        end

    end

    T = T*g_st0;

end