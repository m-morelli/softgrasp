function srv_plot_contact_plane(T)

if isempty(findobj(gcf,'type','axes'))
        ax = axes;
        grid on;
        axis equal;
else
    ax = gca;
end


[x,y,z] = sphere(4);
h = surface(2*x,2*y,z*0.001,'FaceColor','b');
set(h,'EdgeColor','none');
hs = hgtransform('Parent',ax);
set(h,'Parent',hs);
set(hs,'Matrix',T);
hold on;
arrow3(T(1:3,4),T(1:3,1:3)*[0; 0; 1],'green');
hold on;

end