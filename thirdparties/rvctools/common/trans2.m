function T = trans2(x, y, theta)
    if length(x) == 3
        y = x(2);
        theta = x(3);
        x = x(1);
    end
    T = [cos(theta) -sin(theta) x
         sin(theta)  cos(theta) y
         0           0          1 ];
