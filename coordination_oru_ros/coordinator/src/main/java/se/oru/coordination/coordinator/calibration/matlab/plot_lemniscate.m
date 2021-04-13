xmax = 1;
width = xmax; %footprint_x
height = xmax/2;
n = 16;
poses = lemniscate(xmax, width, height, n);

figure;
for i = 1 : size(poses, 2)
    xl = -width/2;
    xr = width/2;
    yb = -height/2;
    yt = height/2;
    poses(3, i) = wrapToPi(poses(3, i)+pi);
    t = poses(3, i);
    poses(1:2, i) = [cos(-pi/2) -sin(pi/2); sin(-pi/2) cos(pi/2)]*poses(1:2, i);
    %Rotate footprint coordinates and traslate them
    p = polyshape([cos(t)*xl-sin(t)*yt+poses(1, i), cos(t)*xr-sin(t)*yt+poses(1, i), cos(t)*xr-sin(t)*yb+poses(1, i), cos(t)*xl-sin(t)*yb+poses(1, i)],...
        [sin(t)*xl+cos(t)*yt+poses(2, i), sin(t)*xr+cos(t)*yt+poses(2, i), sin(t)*xr+cos(t)*yb+poses(2, i), sin(t)*xl+cos(t)*yb+poses(2, i)]); 
    plot(p);
    hold on;
end
plot(poses(1,:), poses(2,:), 'o');
axis equal
