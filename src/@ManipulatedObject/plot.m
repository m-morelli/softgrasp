%%%%  PLOT MANIPULATED OBJECT   %%%%

function plot(mo,T,varargin)


    if nargin < 2,
        error('MANIPULATEDOBJECT:classProps:NonConsistentValue', ...
        'wrong number of rhs arguments, expected a homogeneous transformation');
    end
    
    if nargin > 3
        error('MANIPULATEDOBJECT:classProps:NonConsistentValue', ...
        'too many input arguments');
    end

    % check if mo is a Manipulated Object
    if ~isa(mo, 'ManipulatedObject'),
        error('MANIPULATEDOBJECT:classProps:UnknownInput', ...
            'wrong input argument, expected a ManipulatedObject');
    end
    

    
    if (isa(T, 'numeric'))
        if(size(T,1)==1 && size(T,2) == 3)||(size(T,1)==3 && size(T,2) == 1)
            T = homogeneous(eye(3),T);
        else
            if(size(T,1)==1 && size(T,2) == 6)||(size(T,1)==6 && size(T,2) == 1)
                T = homogeneous(rotx(T(4))*roty(T(5))*rotz(T(6)),T(1:3));
            else
                if(size(T,1)==4 && size(T,2) == 4)
                    % ok, it's a homogeneous transformation
                else
                    error('MANIPULATEDOBJECT:classProps:UnknownProperty', ...
                        'wrong input argument, expected a homogeneous transformation or a single vector 3x1 or 6x1');
                end
            end
        end
    else
        error('MANIPULATEDOBJECT:classProps:UnknownProperty', ...
            'wrong input argument, expected a numeric value');
    end

    
    % check for optional argument
    if nargin > 2,
            if ~isa(varargin{1}, 'GrOptPlot'),
                error('MANIPULATEDOBJECT:classProps:NonConsistentValue', ...
                    'wrong input argument, expected a GrOptPlot object');
            else
                PlotOpt = varargin{1};
            end
    else
        PlotOpt = GrOptPlot;  % builds standard plot options
    end
    
    
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
    else
        ax = gca;
    end

switch(mo.name)

    case 'Sphere'
        
        [x1,y1,z1]=sphere(60);
        h = surface(x1*mo.radius,y1*mo.radius,z1*mo.radius,'FaceColor',PlotOpt.objectColor);
        set(h,'EdgeColor','none');
        hs = hgtransform('Parent',ax);
        set(h,'Parent',hs);
        set(hs,'Matrix',T);
        camlight left;

    case 'Cylinder'

        X = circle([0 0 0],mo.radius,'n',100);

        h = hgtransform('Parent',ax);

        % cylinder body
        [x1,y1,z1]=cylinder(mo.radius,60);
        h1 = surface(x1,y1,(z1*mo.height),'FaceColor',PlotOpt.objectColor);
        set(h1,'EdgeColor','none');
        hs1 = hgtransform('Parent',h);
        T1 = makehgtform('translate',[0,0,-(mo.height)/2]);
        set(h1,'Parent',hs1);
        set(hs1,'Matrix',T1);
        set(hs1,'Parent',h);

        % cylinder base
        h2 = patch(X(1,:)',X(2,:)',X(3,:)','FaceColor',PlotOpt.objectColor);
        set(h2,'EdgeColor','none');
        hs2 = hgtransform('Parent',h);
        T2 = makehgtform('translate',[0,0,-(mo.height)/2]);
        set(h2,'Parent',hs2);
        set(hs2,'Matrix',T2);
        set(hs2,'Parent',h);

        % cylinder top
        h3 = patch(X(1,:)',X(2,:)',X(3,:)','FaceColor',PlotOpt.objectColor);
        set(h3,'EdgeColor','none');
        hs3 = hgtransform('Parent',h);
        T3 = makehgtform('translate',[0,0,(mo.height)/2]);
        set(h3,'Parent',hs3);
        set(hs3,'Matrix',T3);
        set(hs3,'Parent',h);
        
        set(h,'Matrix',T);

    otherwise
        error('MANIPULATEDOBJECT: no plot metod defined for ''%s''', mo.name);
            
end



end

