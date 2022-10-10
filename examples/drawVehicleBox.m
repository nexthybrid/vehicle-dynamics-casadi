function p = drawVehicleBox(X_c,Y_c,psi,L,W,varargin)
            %drawVehicleBox Draw vehicle box shape using line command
            % p = drawVehicleBox(X_c,Y_c,psi,L,W,colors)
            % Drawing vehicle box shape using a combination of line
            % commands, which doesn't require the existing figure to have
            % hold on property.
            % Inputs:
            % X_c, Y_c: 1xn double , vehicle CG X,Y coordinates [m]
            % psi: 1xn double , yaw angle [rad]
            % L: double, vehicle length [m]
            % W: double, vehicle width [m]
            % colors: 4x3 double[0-1], vehicle body edge colors (f,rear,rgt,l)
            % Output:
            % p: 1x4 handles, the line object handles to the four lines.
            
            % calculating vertices of vehicle rectangular box vertices,
            % with subscript order:
            % 1 - Left Front, 2 - Right Front, 3 - Left Rear, 4 - Right
            % Rear, illustrated below:
            % 
            % + 3 --------------------------------------- + 1
            % |                                                     |
            % |                          .                          |
            % |                        CG                        |
            % |                                                     |
            % + 4 --------------------------------------- + 2

            Defaults = {[0 0 0; 0 0 1; 0 1 0; 1 0 0]};
            Defaults(1:nargin-5) = varargin;
            colors = Defaults{1}; % edge colors (front, right, rear, left)

            
            xVert = [L/2    L/2     -L/2    -L/2];
            yVert = [W/2    -W/2    W/2     -W/2];
            
            Verticies = zeros(length(X_c),4,2);  % initialize verticies XY coords
            for i = 1:4
                for j = 1:length(X_c)
                    Verticies(j,i,:) = [cos(psi(j)) -sin(psi(j)); sin(psi(j)) cos(psi(j))] * ...
                        [xVert(i); yVert(i)] + [X_c(j); Y_c(j)];
                end
            end
            
            XVert = Verticies(:,:,1);
            YVert = Verticies(:,:,2);
            for j = 1:length(X_c)
                p(1) = line([XVert(j,1) XVert(j,2)],[YVert(j,1) YVert(j,2)],'Color',colors(1,:),'LineStyle','-');
                p(2) = line([XVert(j,2) XVert(j,4)],[YVert(j,2) YVert(j,4)],'Color',colors(2,:),'LineStyle','-');
                p(3) = line([XVert(j,4) XVert(j,3)],[YVert(j,4) YVert(j,3)],'Color',colors(3,:),'LineStyle','-');
                p(4) = line([XVert(j,3) XVert(j,1)],[YVert(j,3) YVert(j,1)],'Color',colors(4,:),'LineStyle','-');
            end
            axis equal
end


