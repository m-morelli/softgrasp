%TRPLOT Draw a coordinate frame
%
% TRPLOT(T, OPTIONS) draws a 3D coordinate frame represented by the homogeneous 
% transform T.
%
% TRPLOT(R, OPTIONS) draws a 3D coordinate frame represented by the orthonormal
% rotation matrix R.
%
% Options::
% 'color', c         The color to draw the axes, Matlab colorspec.
% 'axes'             Set dimensions of the Matlab axes
% 'frame', f         The name which appears on the axis labels and the frame itself
% 'text_opts', opt   A cell array of Matlab text properties
% 'arrow'            Use arrows rather than line segments for the axes
% 'width', w         Width of arrow tips
% 'handle', h        Draw in the Matlab axes specified by h
%
% See also TRPLOT2, TRANIMATE.


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
%  need to decide how to handle scaling
%  what does hold on mean?  don't touch scaling?

function hout = trplot(T, varargin)

    opt.color = 'b';
    opt.axes = true;
    opt.axis = [];
    opt.frame = [];
    opt.text_opts = [];
    opt.view = [];
    opt.width = 1;
    opt.arrow = false;
    opt.handle = [];

    opt = tb_optparse(opt, varargin);

    if isempty(opt.text_opts)
        opt.text_opts = {};
    end
    if isempty(opt.axis)
        if all(size(T) == [3 3]) || norm(transl(T)) < eps
            opt.axis = [-1 1 -1 1 -1 1]*1.2';
        end
            
    end
    
    % TODO: should do the 2D case as well
    
    if ~isempty(opt.handle)
        hax = opt.handle;
        hold(hax);
        hax
    else
        ih = ishold;
        if ~ih
            % if hold is not on, then clear the axes and set scaling
            cla
            if ~isempty(opt.axis)
                axis(opt.axis);
            end
            daspect([1 1 1]);
            
            if opt.axes
                xlabel( 'X');
                ylabel( 'Y');
                zlabel( 'Z');
                rotate3d on
            end
            new_plot = true;
        else
            %set(gca, 'XLimMode', 'auto');
            %set(gca, 'YLimMode', 'auto');
            %set(gca, 'ZLimMode', 'auto');
        end
        hax = gca;
        hold on
    end

    opt.text_opts = {opt.text_opts{:}, 'Color', opt.color};


    % trplot( Q.R, fmt, color);
    if size(T) == [3 3]
        T = r2t(T);
    end


    % create unit vectors
    o =  homtrans(T, [0 0 0]');
    x1 = homtrans(T, [1 0 0]');
    y1 = homtrans(T, [0 1 0]');
    z1 = homtrans(T, [0 0 1]');

    
    % draw the axes
    
    mstart = [o o o]';
    mend = [x1 y1 z1]';
    
    if isempty(opt.handle)
        hg = hgtransform;
    else
        hg = hax;
    end

    if opt.arrow
        % draw the 3 arrows
        S = [opt.color num2str(opt.width)];
        ha = arrow3(mstart(:,1:3), mend(:,1:3), S);
        for h=ha'
            set(h, 'Parent', hg);
        end
    else
        for i=1:3
            h = plot2([mstart(i,1:3); mend(i,1:3)], 'Color', opt.color, 'Parent', hg);

        end
    end

    % label the axes
    if isempty(opt.frame)
        fmt = '%c';
    else
%             if opt.axlabel(1) == '$'
%                 fmt = axlabel;
%                 opt.text_opts = {opt.text_opts{:}, 'Interpreter', 'latex'};
%             else
%                 fmt = sprintf('%%c%s', axlabel);
%             end
        fmt = sprintf('%%c_{%s}', opt.frame);
    end
    
    % add the labels to each axis
    h = text(x1(1), x1(2), x1(3), sprintf(fmt, 'X'), 'Parent', hg);
    set(h, opt.text_opts{:});
   

    h = text(y1(1), y1(2), y1(3), sprintf(fmt, 'Y'), 'Parent', hg);
    set(h, opt.text_opts{:});
    

    h = text(z1(1), z1(2), z1(3), sprintf(fmt, 'Z'), 'Parent', hg);
    set(h, opt.text_opts{:});
    
    % label the frame
    if ~isempty(opt.frame)
        h = text(o(1)-0.04*x1(1), o(2)-0.04*y1(2), o(3)-0.04*z1(3), ...
            ['\{' opt.frame '\}'], 'Parent', hax);
        set(h, 'VerticalAlignment', 'middle', ...
            'HorizontalAlignment', 'center', opt.text_opts{:});
    end
    
    if ~opt.axes
        set(gca, 'visible', 'off');
    end
    if isstr(opt.view) && strcmp(opt.view, 'auto')
        cam = x1+y1+z1;
        view(cam(1:3));
    elseif ~isempty(opt.view)
        view(opt.view);
    end
    if isempty(opt.handle) && ~ih
        grid on
        hold off
    end
    
    if nargout > 0
        hout = hg;
    end
