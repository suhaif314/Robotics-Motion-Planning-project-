function route = GradientBasedPlanner (f, start_coords, end_coords, iterations)
% this function plans a path through 2D 

[gx, gy] = gradient(-f);

route = start_coords;
Point_on_route = start_coords;
Speed = 3;
Tolerance = 1;

while(iterations >0)
    if(norm(end_coords - Point_on_route)<Tolerance)
        break;
    end
    delta_x = gx(floor(Point_on_route(2)), floor(Point_on_route(1)));
    delta_y = gy(floor(Point_on_route(2)), floor(Point_on_route(1)));

    delta = [delta_x, delta_y];
    %delta vector is both value and direction.

    delta_Direction_x = delta_x/norm(delta);
    delta_Direction_y = delta_y/norm(delta);
    
    new_route_x = Point_on_route(1) + Speed * delta_Direction_x;
    new_route_y = Point_on_route(2) + Speed * delta_Direction_y;
    
    Point_on_route = [new_route_x, new_route_y];

    route = [route; Point_on_route];
    iterations = iterations -1 ;
 
end