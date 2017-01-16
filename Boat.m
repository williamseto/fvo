classdef Boat
    %BOAT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        x
        y
        theta
        vel
        scale
        h_boat
        points
        default_points
        formation_pts = {}
        type
        
        boat_pts = [0.5581 0.5116 0.3721 0.1628 -0.3953 -0.4419 -0.3953 0.1628 0.3721 0.5116
                    0 0.0698 0.1395 0.1860 0.1628 0 -0.1628 -0.1860 -0.1395 -0.0698];
        
        contact_pts = [3.0 3.0 -3.0  -3.0
                      -1.5 1.5  1.5  -1.5];
           
        fat_pts = [-1.5  1.5    1.5   -1.5
                    5.0  5.0   -5.0   -5.0];
                
        skinny_pts = [5.0  5.0 -5.0   -5.0   
                     -1.5  1.5  1.5   -1.5];
                 
        square_pts = [4 4 -4 -4
                     -4 4  4 -4];
                 
        min_pts = [1.5 1.5 -1.5 -1.5
                  -1.5 1.5  1.5 -1.5];

        diamond_pts = [3 0  -3  0
                       0 3   0 -3];
    end
    
    methods
        
        % Constructor
        function obj = Boat(x, y, theta, vel, scale, type, color, transparency)
            if nargin > 1
                obj.x = x;
                obj.y = y;
                obj.theta = theta;
                obj.vel = vel;
                obj.type = type;
                
                switch type
                    case 'boat'
                        obj.default_points = obj.boat_pts;
                    case 'formation'
                        obj.default_points = obj.skinny_pts;
                    case 'contact'
                        obj.default_points = obj.contact_pts;
                    case 'static'
                        obj.default_points = obj.contact_pts;
                    otherwise
                        obj.default_points = obj.contact_pts;
                end
                
                obj.scale = scale;        
                obj.default_points = obj.default_points * scale;
                obj.points = obj.default_points;
                
                obj.h_boat = patch(obj.default_points(1,:), obj.default_points(2,:), color, 'FaceAlpha', transparency);
            end
        end
        
        % Plot
        function obj = plot(obj)
            R = [cos(obj.theta) -sin(obj.theta) 
                 sin(obj.theta)  cos(obj.theta)];

            xy = [obj.x; obj.y];
            obj.points = R * obj.default_points + xy*ones(1,size(obj.points,2));
%             if obj.type ~= 'formation'
                set(obj.h_boat, 'XData', obj.points(1,:), 'YData', obj.points(2,:));
%             end
        end
        
        function obj = update(obj, dt, vel) 
            
            if nargin > 2
                if norm(vel) ~= 0
                    h = [cos(obj.theta); 
                         sin(obj.theta)];

                    dw = .2 * ( vel/norm(vel') ) * [0 -1; 1 0] * h; 

                    obj.vel = norm(vel);
                    obj.theta = obj.theta + dw;
                else
                   obj.vel = 0;
                end
            end
                        
            obj.x = obj.x + obj.vel * cos(obj.theta)*dt;
            obj.y = obj.y + obj.vel * sin(obj.theta)*dt;
            obj.theta = obj.theta;
        end
        
        function obj = update_formation(obj, type)
           switch type
                case 1
                    obj.default_points = obj.fat_pts * obj.scale;
                case 2
                    obj.default_points = obj.skinny_pts * obj.scale;
                case 3'
                    obj.default_points = obj.diamond_pts * obj.scale;
                otherwise
                    disp('ERROR');
            end 
        end
               
        % Getters
        function xy = loc(obj)
           xy = [obj.x obj.y];
        end
        
        % Creates a rotational matrix for body frame
        function R = rotation(obj)
            R = [cos(obj.theta) -sin(obj.theta); 
                 sin(obj.theta)  cos(obj.theta)];
        end
        
        % Returns a velocity vector
        function vel = velocity(obj)
           vel = [obj.vel * cos(obj.theta), obj.vel * sin(obj.theta)];
        end
        
        function pose = pose(obj)
           pose = [obj.x obj.y obj.theta]; 
        end
    end
    
    methods(Static)
        
        function expanded_pts = expand(b1, b2, type, scale)
            expanded_pts = [];
            pts = [];
      
            switch type
                case 1
                    pts = b1.fat_pts * b1.scale;
                case 2
                    pts = b1.skinny_pts * b1.scale;
                case 3
                    pts = b1.diamond_pts * b1.scale;
                case 4
                    pts = b1.square_pts * b1.scale;
                case 5
                    pts = b1.boat_pts * 20;
                case 6
                    pts = b1.min_pts * b1.scale;                 
                otherwise
                    disp('ERROR');
                    pts = b1.fat_pts;
            end
            
            for i = 1:size(b2.points, 2)
                P = b1.rotation() * pts * scale + b2.points(:,i) * ones(1,size(b1.points,2));
                expanded_pts = [expanded_pts; P'];
            end    
        end
        
    end         
end 