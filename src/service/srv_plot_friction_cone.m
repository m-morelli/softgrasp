function h_cone = srv_plot_friction_cone(mu, Cn, scale)

if isempty(findobj(gcf,'type','axes'))
        ax = axes;
        grid on;
        axis equal;
else
    ax = gca;
end

h_cone = hgtransform('Parent',ax);

theta = atand(mu);
%r = tand(theta);

r = scale*5;
l = scale*10;
[x,y,z] = cylinder([r 0]);
h = surface(x,y,z*l,'FaceColor','y');
hl = line([0 0],[0 0],[0 l*2.5],'Linewidth',2);
hs = hgtransform('Parent',ax);
% rotate the cone to match Cn - begin
f = [0;0;-1];
t = abs(Cn);
v = cross(f,t);
u = v/max(sqrt(v'*v),1e-9);
fi = acos(f'*t);
Q = Quaternion(fi, u);
% rotate the cone to match Cn - end
%T = makehgtform('translate',[0,0,l],'yrotate',pi);
T = rt2tr(Q.R, [l,0,0]');
set(h,'Parent',hs);
set(hl,'Parent',h_cone);
set(hs,'Matrix',T);
set(hs,'Parent',h_cone);



end

