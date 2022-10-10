function drawWheels(X_c,Y_c,psi,delta,a,b,varargin)
    %drawWheels Draw four wheels
    % drawWheels(X_c,Y_c,psi,delta,a,b,tw,tireWidth,tireRadius) draws four wheels
    % based on vehicle CG, yaw angle, front steering angle, front and rear axle
    % distance, track width, tire width and radius.
    % Inputs:
    % X_c, Y_c: 1xn double , vehicle CG X,Y coordinates [m]
    % psi: 1xn double , yaw angle [rad]
    % delta: 1xn double, vehicle front axle steering angle [rad]
    % a,b : double, vehicle length [m]
    % tw: double, vehicle track width between left and right wheels [m]
    % tireWidth, tireRadius: double, tire width and tire radius [m]
    % colors: 4x3 double[0-1], vehicle tire edge colors (f,rear,rgt,l)
    % Output:
    % p: 1x4 handles, the line object handles to the four lines.

    % calculating positions of vehicle wheel centers on the body,
    % with subscript order:
    % 1 - Left Front, 2 - Right Front, 3 - Left Rear, 4 - Right Rear,
    % illustrated below:
    % 
    % + 3 --------------------------------------- + 1
    % |                                                     |
    % |                          .                          |
    % |                        CG                        |
    % |                                                     |
    % + 4 --------------------------------------- + 2

    Defaults = {1.6,0.25,0.3,[0 0 0; 0 0 0; 0 0 0; 0 0 0]};
    Defaults(1:nargin-6) = varargin;
    tw = Defaults{1}; % track width [m]
    tireWidth = Defaults{2}; % tire width [m]
    tireRadius = Defaults{3}; % tire radius [m]
    colors = Defaults{4}; % edge colors (front, right, rear, left)
    
    %%% STEP 1 Calculate the wheel center locations
    xVert = [a    a     -b    -b];
    yVert = [tw/2    -tw/2    tw/2     -tw/2];
    % vertices of the wheel centers
    Verticies = zeros(length(X_c),4,2);  % initialize verticies XY coords
    for i = 1:4 % i = 1,2,3,4: four body corners
        for j = 1:length(X_c) % j = 1,...,n: number of poses
            Verticies(j,i,:) = [cos(psi(j)) -sin(psi(j)); sin(psi(j)) cos(psi(j))] * ...
                [xVert(i); yVert(i)] + [X_c(j); Y_c(j)];
        end
    end
    
    XVert = Verticies(:,:,1);
    YVert = Verticies(:,:,2);
    
    %%% STEP 2 Calculate the wheel rubber corner locations
    numWheels = 4;
    xVert = [tireRadius    tireRadius     -tireRadius    -tireRadius];
    yVert = [tireWidth/2    -tireWidth/2    tireWidth/2     -tireWidth/2];
    psiFront = psi+delta;       % front wheel yaw rangle w.r.t. ground, 1xn
    psiRear = psi;                  % rear wheel yaw angle w.r.t. ground, 1xn
    psiWheels = [psiFront; psiFront; psiRear; psiRear]; % 4xn
    Verticies = zeros(length(X_c),numWheels,4,2);  % initialize verticies XY coords
    for j = 1:length(X_c)  % j = 1,...,n: number of poses
        % configure each of the four wheels' four vertices
        for k = 1:4 % k = 1,2,3,4: wheel center locations
            XWhlCntr = XVert(j,k);
            YWhlCntr = YVert(j,k);
            % vertices of the wheel rubber corners, in total nxnumWheelsx4
            % corners, each with two cordinate values
            
            for m = 1:4 % m = 1,2,3,4: four wheel rubber corners
                Verticies(j,m,k,:) = [cos(psiWheels(k,j)) -sin(psiWheels(k,j));...
                    sin(psiWheels(k,j)) cos(psiWheels(k,j))] * ...
                    [xVert(m); yVert(m)] + [XWhlCntr; YWhlCntr];
            end
        end
    end

    XVertTr = Verticies(:,:,:,1); % n x numWheels x 4
    YVertTr = Verticies(:,:,:,2); % n x numWheels x 4
    for j = 1:length(X_c)
        for m = 1:4 % m = 1,2,3,4: four wheel rubber corners
            line([XVertTr(j,1,m) XVertTr(j,2,m)],[YVertTr(j,1,m) YVertTr(j,2,m)],'Color',colors(1,:),'LineStyle','-');
            line([XVertTr(j,2,m) XVertTr(j,4,m)],[YVertTr(j,2,m) YVertTr(j,4,m)],'Color',colors(2,:),'LineStyle','-');
            line([XVertTr(j,4,m) XVertTr(j,3,m)],[YVertTr(j,4,m) YVertTr(j,3,m)],'Color',colors(3,:),'LineStyle','-');
            line([XVertTr(j,3,m) XVertTr(j,1,m)],[YVertTr(j,3,m) YVertTr(j,1,m)],'Color',colors(4,:),'LineStyle','-');
        end
    end

    axis equal

end