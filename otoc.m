function [ gtxr gtyr ] = Untitled( gtx, gty, gttheta )
%otoci data okolo prvnich bodu o uhel gttheta
x = gtx';
y = gty';
v = [x;y];
x_stred = x(1);
y_stred = y(1);
stred = repmat([x_stred; y_stred], 1, length(x));
theta = -gttheta(1);
R = [cos(theta) -sin(theta); sin(theta) cos(theta)];

s = v - stred;
so = R*s;
vo = so + stred;

x_rot = vo(1,:);
y_rot = vo(2,:);

gtxr = x_rot';
gtyr = y_rot';
end

