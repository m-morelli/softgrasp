function srv_plot_core(ax, links_fkine, joints_fkine, base, PlotOpt, joints)

    linknum = size(links_fkine,3);
    scaling = PlotOpt.scaling;
    
    ja = 0; % show joint axes, or not
    if strcmp(PlotOpt.jointsAxis,'on')
        ja = 1;
    end
        
    LinksHandlesVect = zeros(linknum,1);
    JointsHandlesVect = zeros(linknum,1);
    terminal = 0;
    
    for k = 1 : linknum
        
        if(k == 1)
            length_i = norm(links_fkine(1:3,4,k)-base);
            rotrasl_link = rt2tr(links_fkine(1:3,1:3,k),base);
        else
            length_i = norm(links_fkine(1:3,4,k)-links_fkine(1:3,4,k-1));
            rotrasl_link = rt2tr(links_fkine(1:3,1:3,k),links_fkine(1:3,4,k-1));
        end

        if(k == linknum)
            terminal = 1;
        end
                    
        rotrasl_joint = joints_fkine(:,:,k);
        LinksHandlesVect(k)= srv_makelink(ax,PlotOpt.linksRadius/scaling, ... 
                                          PlotOpt.linksColor,length_i,terminal,joints(k));
        set(LinksHandlesVect(k),'Matrix',rotrasl_link);
        JointsHandlesVect(k) = srv_makejoint(ax,PlotOpt.jointsRadius, ... 
                                       2.3*PlotOpt.linksRadius,PlotOpt.jointsColor,scaling);
        set(JointsHandlesVect(k),'Matrix',rotrasl_joint);
                
        if(ja)
            hold on;
            srv_jointaxis(ax,rotrasl_joint(1:3,4), (4/scaling)*rotrasl_joint(1:3,3),'k',2);
        end
              
    end
end

function h_obj = srv_makelink(ax, radius, link_color, length, terminal, prismatic)

nf = 60;  % facets number
axis_handle = ax;
h_obj = hgtransform('Parent',axis_handle);
if(length)
if(prismatic)
    
    [x1,y1,z1]=cylinder(radius,nf);
    h1 = surface(x1,y1,z1*length/2,'FaceColor',link_color);
    set(h1,'EdgeColor','none');
    set(h1,'Parent',h_obj);
   
    [x2,y2,z2] = cylinder(radius*0.8,6);
    if(terminal)
        h2 = surface(x2,y2,z2*(length/2 - radius),'FaceColor',link_color);
    else
        h2 = surface(x2,y2,z2*length/2,'FaceColor',link_color);
    end
    
    set(h2,'EdgeColor','none');
    hs2 = hgtransform('Parent',axis_handle);
    T = makehgtform('translate',[0,0,length/2]);
    set(h2,'Parent',hs2);
    set(hs2,'Matrix',T);
    set(hs2, 'Parent', h_obj);
   
    [x3,y3,z3]=cylinder([radius 0]);
    h3 = surface(x3,y3,z3*0.01,'FaceColor',link_color);
    set(h3,'EdgeColor','none');
    hs3 = hgtransform('Parent',axis_handle);
    T1 = makehgtform('translate',[0,0,length/2]);
    set(h3,'Parent',hs3);
    set(hs3,'Matrix',T1);
    set(hs3, 'Parent', h_obj);

else

    [x1,y1,z1]=cylinder(radius,nf);
    if(terminal)
        h1 = surface(x1,y1,z1*(length - radius),'FaceColor',link_color);
    else
        h1 = surface(x1,y1,z1*length,'FaceColor',link_color);
    end
        
    set(h1,'EdgeColor','none');
    set(h1,'Parent',h_obj);  
    
end

if(terminal)
    [x3,y3,z3]=sphere(nf);
    z3(z3<0)=0;
    h3 = surface(radius*x3,radius*y3,radius*z3,'FaceColor',link_color);
    set(h3,'EdgeColor','none');
    hs=hgtransform('Parent',axis_handle);
    T=makehgtform('translate',[0,0,length-radius]);
    set(h3, 'Parent', hs);
    set(hs,'Matrix',T);
    set(hs,'Parent',h_obj);
end
end
end

function h_obj = srv_makejoint(ax, radius, height, joint_col, scaling)
    
nf = 60; % number of facets

X = circle([0 0 0],(radius+.25)/scaling,'n',100);

axis_handle = ax;
h_obj = hgtransform('Parent',axis_handle);

% joint body
[x1,y1,z1]=cylinder((radius+(0.25))/scaling,nf);
h1 = surface(x1,y1,(z1*height)/scaling,'FaceColor',joint_col);
set(h1,'EdgeColor','none');
hs1 = hgtransform('Parent',axis_handle);
T = makehgtform('translate',[0,0,-(height/2)/scaling]);
set(h1,'Parent',hs1);
set(hs1,'Matrix',T);
set(hs1,'Parent',h_obj);

% joint lateral facets
h2 = patch(X(1,:)',X(2,:)',X(3,:)','FaceColor',joint_col);
set(h2,'EdgeColor','none');
hs2 = hgtransform('Parent',axis_handle);
T = makehgtform('zrotate',pi/2,'translate',[0,0,(height/2)/scaling]);
set(h2,'Parent',hs2);
set(hs2,'Matrix',T);
set(hs2,'Parent', h_obj); 


h3 = patch(X(1,:)',X(2,:)',X(3,:)','FaceColor',joint_col);
set(h3,'EdgeColor','none');
hs3 = hgtransform('Parent',axis_handle);
T = makehgtform('zrotate',pi/2,'translate',[0,0,-(height/2)/scaling]);
set(h3,'Parent',hs3);
set(hs3,'Matrix',T);
set(hs3,'Parent', h_obj); 

end

function [] = srv_jointaxis(ax,O, D, color, linewidth)
%ARROW3  plots a 3d arrow
%
%	ARROW3(O, D)
%	ARROW3(O, D, COLOR, LINEWIDTH)
%
% Draws an arrow with the origin at O, and in direction D.  Thus, the head
% of the arrow will be at O + D.

if nargin < 4
    color = 'k';
end

if nargin < 5
    linewidth = 2;
end

if ~isequal([3 1], size(O)) || ~isequal([3 1], size(D))  
  error('SCREWS:arrow3', 'requires 3x1 input vector');
end

tail_length = 10;
tail_width = 2;

len = norm(D);

off = len/tail_length;

if 0 == len,
  return;
end

% the arrow points along the x-axis for now
points = [0 0 0; ...
          len 0 0; ...
          len-off -off/tail_width 0; ...
          len-off off/tail_width 0; ...
          len 0 0;]';

% build a rotation matrix to make our lives easier
R(:,1) = D/len;
R(:,2) = roty(pi/2)*D/len;
if (max(sum(R(:, 1:2).^2, 2)) > 1)
  R(:,2) = rotx(pi/2)*D/len;  
end
R(:,3) = sqrt(ones(3,1) - R(:, 1).^2 - R(:,2).^2) ;

% rotate the points
points = R * points + repmat(O, 1, size(points,2));

hchek = ishold;

% plot everything
plot3(ax,points(1, 1:2), points(2, 1:2), points(3, 1:2), ...
      'color', color, 'linewidth', linewidth);
hold on;
h = patch(points(1, 2:end)', points(2, 2:end)', points(3, 2:end)', color);
set(h, 'LineStyle', 'none');

if 0 == hchek
   hold off
end
end