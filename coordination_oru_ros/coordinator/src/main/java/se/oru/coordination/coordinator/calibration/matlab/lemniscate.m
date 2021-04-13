function poses = lemniscate(xmax, width, height, n)
%LEMNISCATE Summary of this function goes here
%   Detailed explanation goes here
    tvec = linspace(-pi, pi, n);%union(linspace(-pi/4, pi/4, n/2), linspace(3*pi/4, 5*pi/4, n/2));
    poses = zeros(3, length(tvec));
    for i = 1 : length(tvec)
        t = tvec(i);
        x = xmax*cos(t)/(1+sin(t)^2);
        y = x*sin(t);
        theta = atan2((y*(1+2*x^2+2*y^2)), x*(1-2*x^2-2*y^2));
        poses(1:3, i) = [x; y; theta];
    end
end

