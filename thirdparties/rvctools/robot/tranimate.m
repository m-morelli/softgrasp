%TRANIMATE Animate a coordinate frame
%
% TRANIMATE(P1, P2, OPTIONS) animates a coordinate frame moving from pose P1
% to pose P2.  Pose can be represented by:
%   - homogeneous transformation matrix 4x4
%   - orthonormal rotation matrix 3x3
%   - Quaternion
%
% TRANIMATE(P, OPTIONS) animates a coordinate frame moving from the identity pose
% to the pose P represented by any of the types listed above.
%
% TRANIMATE(PS, OPTIONS) animates a trajectory, where PS is any of
%   - homogeneous transformation matrix sequence 4x4xn
%   - orthonormal rotation matrix sequence 3x3xn
%   - quaternion array n
%
% Options::
%  'fps', fps    Number of frames per second to display (default 10)
%  'nsteps', n   The number of steps along the path (default 50)
%
% See also TRPLOT.

% Copyright (C) 1993-2011, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for Matlab (RTB).
% 
% RTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% RTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with RTB.  If not, see <http://www.gnu.org/licenses/>.

% TODO
%  auto detect the axis scaling
function tranimate(P2, varargin)

    opt.fps = 10;
    opt.nsteps = 50;

    [opt, args] = tb_optparse(opt, varargin);

    P1 = [];

    % convert quaternion and rotation matrix to hom transform
    if isa(P2, 'Quaternion')
        T2 = P2.T;   % convert quaternion to transform
        if ~isempty(args) && isa(args{1},'Quaternion')
            T1 = T2;
            Q1 = args{1};
            T2 = Q1.T;
            args = args(2:end);
        else
            T1 = eye(4,4);
        end
    elseif isrot(P2)
        T2 = r2t(P2);
        if ~isempty(args) && isrot(args{1})
            T1 = T2;
            T2 = r2t(args{1});
            args = args(2:end);
        else
            T1 = eye(4,4);
        end
    elseif ishomog(P2)
        T2 = P2;
        if ~isempty(args) && ishomog(args{1})
            T1 = T2;
            T2 = args{1};
            args = args(2:end);
        else
            T1 = eye(4,4);
        end
    end
        
    if size(P2,3) > 1
        % tranimate(Ts)
        % we were passed a homog sequence
        if ~isempty(P1)
            error('only 1 input argument if sequence specified');
        end
        Ttraj = T2;
    else
        % tranimate(P1, P2)
        % create a path between them
        Ttraj = ctraj(T1, T2, opt.nsteps);
    end

    % do axis checking in here
    %compute coords of origin and ends of axes for start and end pose.
    %T0 = Ttraj(:,:,1)
    %T1 = Ttraj(:,:,end)
    %pts = [0 0 0; 1 0 0; 0 1 0; 0 0 1]';
    %ptsx = [transformp(T0, pts) transformp(T1, pts)];
    %mn = min(ptsx');
    %mx = max(ptsx');
    %axis([mn(1) mx(1) mn(2) mx(2) mn(3) mx(3)]);

    hg = trplot(Ttraj(:,:,1), args{:});

    for i=1:size(Ttraj,3)
        T = Ttraj(:,:,i);
        set(hg, 'Matrix', T);
        pause(1/opt.fps);
    end
