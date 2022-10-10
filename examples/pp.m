classdef pp
    %pp A collection of post-processing functions
    %   This class contains post-processing function relevant to the
    %   vehicle collision avoidance simulations. For example, functions to
    %   draw vehicle shapes and make videos.


    methods (Static)

        function make_video_rectangles(OCP,sol,videoName,foundSol)
            %make_video_rectangles Make a video with the name videoName
            %   make_video_rectangles(OCP,sol,videoName)
            if ~foundSol
                sol = OCP.opti.debug;
            end
            if isempty(OCP.px_obs)
                staticObs = true;
            else
                staticObs = false;
            end
            v = VideoWriter([videoName '.avi']);
            v.FrameRate = 10;
            open(v);
            hFigure = figure();
            hold on;
            X_c_arr = sol.value(OCP.px);
            Y_c_arr = sol.value(OCP.py);
            psi_arr = sol.value(OCP.psi);
            if isprop(OCP,'delta')
                hasWheels = true;
                delta_arr = sol.value(OCP.delta);
            end
            a = OCP.a;
            b = OCP.b;
            Xobs_c = sol.value(OCP.x_b); % static obstacle
            Yobs_c = sol.value(OCP.y_b);
            psiobs_c = 0;
            Xobs_c_arr = sol.value(OCP.px_obs); % dynamic obstacle
            Yobs_c_arr = sol.value(OCP.py_obs);
            psiobs_c_arr = sol.value(OCP.psi_obs);
            
            obs_colors = zeros(4,3); % obstacle color: all black

            for i = 1:length(sol.value(OCP.px))
                plot(sol.value(OCP.px),sol.value(OCP.py),'LineWidth',2);
                hold on;
                drawVehicleBox(X_c_arr(i),Y_c_arr(i),psi_arr(i),OCP.a+OCP.b+OCP.R_col,OCP.R_col);
                if hasWheels
                    drawWheels(X_c_arr(i),Y_c_arr(i),psi_arr(i),delta_arr(i),a,b);
                end
                if staticObs
                    drawVehicleBox(Xobs_c,Yobs_c,psiobs_c,OCP.aBmpr+OCP.bBmpr,OCP.bdyWdth,obs_colors);
                else
                    drawVehicleBox(Xobs_c_arr(i),Yobs_c_arr(i),psiobs_c_arr(i),OCP.aBmpr+OCP.bBmpr,OCP.bdyWdth,obs_colors);
                end
                xlim([-5, 25]);
                ylim([-5, 15]);
                xlabel('X'); ylabel('Y')
                axis equal;
                frame = getframe(hFigure);
                writeVideo(v,frame);
                hold off;
                %ClearLinesFromAxes(); % delete all line objects
            end
            close(v);
            close(hFigure);
            clear hFigure;
            disp(['Video saved as: ', pwd , filesep, videoName ,'.avi'])
        end

        function make_video_circles(OCP,sol,videoName,foundSol)
            %make_video_circles Make a video with the name videoName
            %   make_video_circles(OCP,sol,videoName,foundSol)
            if ~foundSol
                sol = OCP.opti.debug;
            end
            if isempty(OCP.px_obs)
                staticObs = true;
            else
                staticObs = false;
            end

            v = VideoWriter([videoName '.avi']);
            v.FrameRate = 10;
            open(v);
            hFigure = figure();
            hold on;
            X_c_arr = sol.value(OCP.px);
            Y_c_arr = sol.value(OCP.py);
            psi_arr = sol.value(OCP.psi);
            Xobs_c = sol.value(OCP.x_b); % static obstacle
            Yobs_c = sol.value(OCP.y_b);
            psiobs_c = 0;
            Xobs_c_arr = sol.value(OCP.px_obs); % moving obstacle
            Yobs_c_arr = sol.value(OCP.py_obs);
            psiobs_c_arr = sol.value(OCP.psi_obs);
            
            obs_colors = zeros(4,3); % obstacle color: all black
            for i = 1:length(sol.value(OCP.px))
                plot(sol.value(OCP.px),sol.value(OCP.py),'LineWidth',2);
                hold on;
                drawVehicleCircles(X_c_arr(i),Y_c_arr(i),psi_arr(i),OCP.a,OCP.R_col/2);
                if staticObs
                    drawVehicleCircles(Xobs_c,Yobs_c,psiobs_c,...
                        OCP.a,OCP.R_col/2,obs_colors);  
                else
                    drawVehicleCircles(Xobs_c_arr(i),Yobs_c_arr(i),psiobs_c_arr(i),...
                        OCP.a,OCP.R_col/2,obs_colors);  
                end
                xlim([-5, 25]);
                ylim([-5, 15]);
                xlabel('X'); ylabel('Y')
                axis equal;
                frame = getframe(hFigure);
                writeVideo(v,frame);
                hold off;
                %ClearLinesFromAxes(); % delete all line objects
            end 
            close(v);
            close(hFigure);
            clear hFigure;
            disp(['Video saved as: ', pwd , filesep, videoName ,'.avi'])
        end

        function basic_plots(OCP,sol)
            %basic_plots Plot the basic system states: velocity, position, yaw
            %rate, BEV (bird eye view) trajectory
            if isempty(OCP.px_obs)
                staticObs = true;
            else
                staticObs = false;
            end
            figure('Position',[100 100 800 400]);
            tiledlayout(2,2)
            nexttile;
            hold on
            plot(sol.value(OCP.vx),'LineWidth',2);
            plot(sol.value(OCP.vy),'LineWidth',2);
            legend('vx','vy');
            title('Ego velocity')
            nexttile;
            hold on
            plot(sol.value(OCP.px),'LineWidth',2);
            plot(sol.value(OCP.py),'LineWidth',2);
            legend('px','py')
            title('Ego vehicle CG position');
            nexttile;
            hold on
            plot(sol.value(OCP.psi),'LineWidth',2);
            legend('psi')
            title('Ego vehicle yaw rate')
            nexttile;
            plot(sol.value(OCP.px),sol.value(OCP.py),'LineWidth',2);
            xlabel('X'); ylabel('Y')
            obs_colors = zeros(4,3); % obstacle color: all black
            Xobs_c = sol.value(OCP.x_b); % static obstacle
            Yobs_c = sol.value(OCP.y_b);
            psiobs_c = 0;
            Xobs_c_arr = sol.value(OCP.px_obs); % moving obstacle
            Yobs_c_arr = sol.value(OCP.py_obs);
            psiobs_c_arr = sol.value(OCP.psi_obs);
            if staticObs
                drawVehicleBox(Xobs_c,Yobs_c,psiobs_c,...
                    OCP.aBmpr+OCP.bBmpr,OCP.bdyWdth,obs_colors);
            else
                for i = 1:length(OCP.px_obs)
                    drawVehicleBox(Xobs_c_arr(i),Yobs_c_arr(i),psiobs_c_arr(i),...
                        OCP.aBmpr+OCP.bBmpr,OCP.bdyWdth,obs_colors);
                end
            end
            axis equal
        end

        function basic_plots_dugoff(OCP,sol)
            %basic_plots_dugoff Plot the basic system states with the
            %dugoff tire model
        
            figure('Position',[100 100 800 600]);
            tiledlayout(3,2)
            nexttile;
            hold on
            plot(sol.value(OCP.vx),'LineWidth',2);
            plot(sol.value(OCP.vy),'LineWidth',2);
            legend('$v_{x}$','$v_{y}$','interpreter','latex');
            title('Ego velocity in Global XY Coordinates')
            nexttile;
            hold on
            plot(sol.value(OCP.px),'LineWidth',2);
            plot(sol.value(OCP.py),'LineWidth',2);
            legend('$p_x$','$p_y$','interpreter','latex')
            title('Ego vehicle CG position');
            nexttile;
            hold on
            plot(sol.value(OCP.psi),'LineWidth',2);
            legend('$\psi$','interpreter','latex')
            title('Ego vehicle yaw rate')
            nexttile;
            plot(sol.value(OCP.px),sol.value(OCP.py),'LineWidth',2);
            xlabel('X'); ylabel('Y'); axis equal; grid on;
            title('Birdeye View');
            obs_colors = zeros(4,3); % obstacle color: all black
            Xobs_c_arr = sol.value(OCP.px_obs);
            Yobs_c_arr = sol.value(OCP.py_obs);
            psiobs_c_arr = sol.value(OCP.psi_obs);
            for i = 1:length(OCP.px_obs)
                drawVehicleBox(Xobs_c_arr(i),Yobs_c_arr(i),psiobs_c_arr(i),OCP.a+OCP.b,OCP.R_col,obs_colors);
            end
            nexttile;
            plot(sol.value(OCP.omega_f),'LineWidth',2);
            hold on
            plot(sol.value(OCP.omega_r),'LineWidth',2);
            legend('$\omega_f$','$\omega_r$','interpreter','latex');
            nexttile;
            plot(sol.value(OCP.delta),'LineWidth',2);
            legend('$\delta$','interpreter','latex');
            grid on;
        end
        
        function controls_plot(OCP,sol)
            %controls_plot Plot the control signals
            figure('Position',[100 100 800 200])
            tiledlayout(1,2);
            nexttile
            plot(sol.value(OCP.f_thr),'LineWidth',2);
            hold on;
            plot(sol.value(OCP.f_lat),'LineWidth',2);
            title('Longitudinal and Lateral Force Input');
            legend('$f_{thr}$','$f_{lat}$','interpreter','latex')
            nexttile;
            plot(sol.value(OCP.u_beta),'LineWidth',2);
            legend('$u_{\beta}$','interpreter','latex');
            title('Sideslip Control Input');
        end

        function controls_plot_dugoff(OCP,sol)
            %controls_plot_dugoff Plot the control signals in the dugoff
            %model with bicycle body model
            figure('Position',[100 100 800 200])
            tiledlayout(1,2);
            nexttile
            plot(sol.value(OCP.nu),'LineWidth',2);
            title('Steering Input');
            legend('$\nu [rad/s]$','interpreter','latex')
            nexttile;
            plot(sol.value(OCP.tau),'LineWidth',2);
            title('Torque Input')
            legend('$\tau [Nm]$','interpreter','latex');
            title('Torque Input');
        end

        function rectangle_overlap(OCP,sol)
            %rectangle_overlap Draw the ego-obstacle vehicles rectangles in overlap
            if isempty(OCP.px_obs)
                staticObs = true;
            else
                staticObs = false;
            end
            if isprop(OCP,'delta') % check if OCP has the property delta
                hasWheels = true;
                delta_arr = sol.value(OCP.delta);
            else
                hasWheels = false;
            end
            figure();
            hold on;
            plot(sol.value(OCP.px),sol.value(OCP.py),'LineWidth',2);
            X_c_arr = sol.value(OCP.px);
            Y_c_arr = sol.value(OCP.py);
            psi_arr = sol.value(OCP.psi);
            Xobs_c = sol.value(OCP.x_b); % static obstacle
            Yobs_c = sol.value(OCP.y_b);
            psiobs_c = 0;
            Xobs_c_arr = sol.value(OCP.px_obs); % moving obstacle
            Yobs_c_arr = sol.value(OCP.py_obs);
            psiobs_c_arr = sol.value(OCP.psi_obs);
            
            obs_colors = zeros(4,3); % obstacle color: all black
            drawVehicleBox(X_c_arr,Y_c_arr,psi_arr,OCP.aBmpr+OCP.bBmpr,OCP.bdyWdth);
            if staticObs
                drawVehicleBox(Xobs_c,Yobs_c,psiobs_c,...
                    OCP.aBmpr+OCP.bBmpr,OCP.bdyWdth,obs_colors);
            else
                drawVehicleBox(Xobs_c_arr,Yobs_c_arr,psiobs_c_arr,...
                    OCP.aBmpr+OCP.bBmpr,OCP.bdyWdth,obs_colors);
            end
            if hasWheels
                drawWheels(X_c_arr,Y_c_arr,psi_arr,delta_arr,OCP.a,OCP.b);
            end
            
            xlabel('X'); ylabel('Y')
            axis equal
        end
        
        function circles_overlap(OCP,sol)
            %circles_overlap Draw the ego-obstacle vehicles Ziegler 3-circles in overlap
            if isempty(OCP.px_obs)
                staticObs = true;
            else
                staticObs = false;
            end
            figure();
            hold on;
            plot(sol.value(OCP.px),sol.value(OCP.py),'LineWidth',2);
            X_c_arr = sol.value(OCP.px);
            Y_c_arr = sol.value(OCP.py);
            psi_arr = sol.value(OCP.psi);
            Xobs_c = sol.value(OCP.x_b); % static obstacle
            Yobs_c = sol.value(OCP.y_b);
            psiobs_c = 0;
            Xobs_c_arr = sol.value(OCP.px_obs); % moving obstacle
            Yobs_c_arr = sol.value(OCP.py_obs);
            psiobs_c_arr = sol.value(OCP.psi_obs);
            
            obs_colors = zeros(3,3); % obstacle color: all black
            if staticObs
                for i = 1:length(sol.value(OCP.px))
                    drawVehicleCircles(X_c_arr(i),Y_c_arr(i),psi_arr(i),OCP.a,OCP.R_col/2);
                end
                drawVehicleCircles(Xobs_c,Yobs_c,psiobs_c,...
                    OCP.a,OCP.R_col/2,obs_colors);
            else
                for i = 1:length(sol.value(OCP.px))
                    drawVehicleCircles(X_c_arr(i),Y_c_arr(i),psi_arr(i),OCP.a,OCP.R_col/2);
                    drawVehicleCircles(Xobs_c_arr(i),Yobs_c_arr(i),psiobs_c_arr(i),OCP.a,OCP.R_col/2,obs_colors);
                end
            end
            
            xlabel('X'); ylabel('Y')
            axis equal
        end
    end

end