function plot_vehicle(x, varargin)
    a = axis;
    if isnumeric(varargin{1})
        d = varargin{1};
        varargin = varargin(2:end);
    else
        d = (a(2)+a(4) - a(1)-a(3))/60;
    end
    points = [
        d 0 1
        -d -0.6*d 1
        -d 0.6*d 1
        d 0 1];
    T = se2(x(1), x(2), x(3));
    points = T * points';
    plot(points(1,:), points(2,:), varargin{:});
