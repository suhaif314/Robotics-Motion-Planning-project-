function plot_a_circle = circle(x,y,r)
theta = linspace(0,2*pi,100);
xunit = r*cos(theta) +x;
yunit = rsin(theta)+ y;
plot_a_circle = plot(xunit,yunit);

end