classdef colliOCP < handle
    %colliOCP A class for storing the casadi.Opti() object for solving the
    %collision avoidance OCP.
    %   Prior to instantiate an object of this class, the casadi package
    %   should be imported by calling:
    %   addpath('C:\Program Files\casadi-windows-matlabR2016a-v3.5.5');
    %   import casadi.*

    properties % casadi class object
        opti
    end

    properties % decision variables
        X
        vx
        vy
        px
        py
        psi

        U
        f_thr
        f_lat
        u_beta
    end

    properties % more intermediate symbolic variables
        x_1
        x_2
        x_3
        y_1
        y_2
        y_3

        x_a
        x_b
        x_c
        y_a
        y_b
        y_c

        Xobs
        px_obs
        py_obs
        psi_obs
    end

    properties % parameters with numerical values
        T % final time [s]
        N % number of control intervals
        m % vehicle mass [kg]
        a % ego vehicle front Ziegler circle to CG distance
        b % ego vehicle rear Ziegler circle to CG distance
        R_col % collision radius [m]
        aBmpr % ego vehicle CG to front bumper distance
        bBmpr % ego vehicle CG to rear bumper distance
        bdyWdth % ego vehicle body width
    end

    methods
        function obj = colliOCP(varargin)
            %colliOCP Construct an instance of this class
            %   colliOCP(N) constructs an instance of colliOCP with N
            %   control intervals
            %   The casadi.Opti() object is self-created in the constructor
            % Inputs:
            % N: double, the number of control intervals
            Defaults = {20};
            Defaults(1:nargin-1) = varargin;
            obj.N = Defaults{1};

            obj.opti = casadi.Opti();
        end

        function build_basics(obj)
            %build_basics Build the basic setup of the OCP
            %   build_basics(obj) builds the basic setup of the OCP
            %   including the decision variables, the valued parameters,
            %   the objective function, and the (ego) dynamics constraints.
            obj.build_decision_var();
            obj.add_parameters();
            obj.add_objective();
            obj.add_dynamic_constraints();
        end
        
        function build_decision_var(obj,varargin)
            %build_decision_var Builds decision variables
            %   build_decision_var(obj) builds the symbolic decision
            %   variables and store them as class properties
            
            obj.X = obj.opti.variable(5,obj.N+1); % state trajectory
            obj.vx = obj.X(1,:);
            obj.vy = obj.X(2,:);
            obj.px = obj.X(3,:);
            obj.py = obj.X(4,:);
            obj.psi = obj.X(5,:);
            
            obj.U = obj.opti.variable(3,obj.N);
            obj.f_thr = obj.U(1,:); % [N]
            obj.f_lat = obj.U(2,:); % [N]
            obj.u_beta = obj.U(3,:); % (rad/s)

        end

        function add_parameters(obj,varargin)
            %add_parameters Adds parameters to the OCP
            %   add_parameters(obj,T,m) adds parameters to the OCP, the
            %   current parameters include simulation end time T, and
            %   vehicle mass m.
            % Inputs:
            % T: double, simulation end time [s]
            % m: double, vehicle mass [kg]
            % a: double, ego vehicle front Ziegler circle to CG distance [m]
            % b: double, ego vehicle rear Ziegler circle to CG distance [m]
            % R_col: double, collision radius [m]
            % aBmpr: double, ego vehicle CG to front bumper distance [m]
            % bBmpr: double, ego vehicle CG to rear bumper distance [m]
            % bdyWdth: double, ego vehicle body width [m]

            Defaults = {2,1700,1.5,1.5,2,2,2,2};
            Defaults(1:nargin-1) = varargin;
            obj.T = Defaults{1};
            obj.m = Defaults{2};
            obj.a = Defaults{3};
            obj.b = Defaults{4};
            obj.R_col = Defaults{5};
            obj.aBmpr = Defaults{6};
            obj.bBmpr = Defaults{7};
            obj.bdyWdth = Defaults{8};
        end

        function add_objective(obj)
            %add_objective Adds the objective function to the OCP
            %   add_objective(obj) applies the pre-defined OCP objective
            %   function.
            obj.opti.minimize(sum((obj.f_thr/1000).^2+(obj.f_lat/1000).^2+obj.u_beta.^2));
        end

        function zero_objective(obj)
            %zero_objective Force the objective to be a constant of zero
            % zero_objective Use constant zero as the objective,
            % effectively making no cost functions. So the solver will only
            % need to deal with the feasibility problem, and somehow
            % optimize itself towards a feasible solution.

            obj.opti.minimize(0);
            disp('Zero objective function applied!');
        end

        function add_dynamic_constraints(obj)
            %add_dynamic_constraints Adds the (ego) dynamics constraints
            %   add_dynamic_constraints(obj) adds the ego vehicle dynamics
            %   as equality constraints to the OCP.
            
%             f = @(x,u) [u(1)/obj.m;
%                 u(2)/obj.m;
%                 x(1).*cos(x(5))-x(2).*sin(x(5));...
%                 x(1).*sin(x(5))+x(2).*cos(x(5));...
%                 (x(1).*u(2)-x(2).*u(1))./obj.m./(x(1).^2+x(2).^2)-u(3)]; % dx/dt = f(x,u)
            f = @obj.vehdyn; % we keep the dynamics in a separate function for neatness.

            dt = obj.T/obj.N; % length of a control interval
            for k=1:obj.N % loop over control intervals
               % Runge-Kutta 4 integration
               k1 = f(obj.X(:,k),         obj.U(:,k));
               k2 = f(obj.X(:,k)+dt/2*k1, obj.U(:,k));
               k3 = f(obj.X(:,k)+dt/2*k2, obj.U(:,k));
               k4 = f(obj.X(:,k)+dt*k3,   obj.U(:,k));
               x_next = obj.X(:,k) + dt/6*(k1+2*k2+2*k3+k4); 
               obj.opti.subject_to(obj.X(:,k+1)==x_next); % close the gaps
            end
        end

        function add_control_bounds(obj,varargin)
            %add_control_bounds Add the control bounds to the OCP
            %   add_control_bounds(obj,B_f_thr, B_f_lat, B_u_beta) adds the
            %   lower and upper bounds for the manipulated control inputs
            %   Inputs:
            %   B_f_thr: 1x2 double, lower and upper bounds for f_thr
            %   B_f_lat: 1x2 double, lower and upper bounds for f_lat
            %   B_u_beta: 1x2 double, lower and upper bounds for u_beta

            Defaults = {[-3000, 3000], [-3000, 3000], [-1, 1]};
            Defaults(1:nargin-1) = varargin;
            B_f_thr = Defaults{1};
            B_f_lat = Defaults{2};
            B_u_beta = Defaults{3};

            obj.opti.subject_to(B_f_thr(1)<=obj.f_thr<=B_f_thr(2));
            obj.opti.subject_to(B_f_lat(1)<=obj.f_lat<=B_f_lat(2));
            obj.opti.subject_to(B_u_beta(1)<=obj.u_beta<=B_u_beta(2));
        end

        function add_initial_conditions(obj,varargin)
            %add_initial_conditions Add initial conditions for decision
            %variables
            %   add_initial_conditions(obj,vx0,vy0,px0,py0,psi0) adds the
            %   initial condition for the set of decision variables
            %   selected. In this case, the set is the ego vehicle states.
            %   Inputs:
            %   vx0: double, ego vehicle initial longitudinal speed [m]
            %   vy0: double, ego vehicle initial lateral speed [m]
            %   px0: double, ego vehicle initial global x coordinate [m]
            %   py0: double, ego vehicle initial global y coordinate [m]
            %   psi0: double, ego vehicle initial yaw angle [rad]

            Defaults = {9, -4.5, 0, 5, 0};
            Defaults(1:nargin-1) = varargin;
            vx0 = Defaults{1};
            vy0 = Defaults{2};
            px0 = Defaults{3};
            py0 = Defaults{4};
            psi0 = Defaults{5};

            obj.opti.subject_to(obj.vx(1)==vx0);
            obj.opti.subject_to(obj.vy(1)==vy0);
            obj.opti.subject_to(obj.px(1)==px0);
            obj.opti.subject_to(obj.py(1)==py0);
            obj.opti.subject_to(obj.psi(1)==psi0);
        end

        function add_terminal_conditions(obj,varargin)
            %add_terminal_conditions Add terminal conditions for decision
            %variables
            %   add_terminal_conditions(obj,B_py) adds the terminal
            %   conditions for the decision variables selected. In this
            %   case, the selected decision variable is simply the global y
            %   coordinate of the ego vehicle.

            obj.opti.subject_to(2<=obj.py(obj.N+1)); % lower bound for py
            obj.opti.subject_to(obj.py(obj.N+1)<=7); % upper bound for py
        end

        function add_path_constraints(obj)
            %add_path_constraints Add the path constraints representing the
            %road geometry
            %   add_path_constraints(obj) implements pre-defined road
            %   geometry constraints on the vehicle state. A first version
            %   here is simply imposing the ego vehicle CG constraints

            obj.opti.subject_to(0<=obj.px<=20); % X limit
            obj.opti.subject_to(2<=obj.py<=7); % Y limit
        end

        function add_ego_centers(obj)
            %add_ego_centers Add the ego vehicle collision centers
            %   add_ego_centers(obj) adds the ego vehicle Ziegler collision 
            %   circle centers to the OCP

            % the ego vehicle is denoted 1,2,3 for its three circle centers

            obj.x_1 = obj.px - obj.b.*cos(obj.psi);
            obj.y_1 = obj.py - obj.b.*sin(obj.psi);
            obj.x_2 = obj.px;
            obj.y_2 = obj.py;
            obj.x_3 = obj.px + obj.a.*cos(obj.psi);
            obj.y_3 = obj.py + obj.a.*sin(obj.psi);
        end

        function add_obstacle_centers(obj,varargin)
            %add_obstacle_centers Add the obstacle collision centers
            %   add_obstacle_centers(obj,x_obs_c,y_obs_c,psi_obs_c) adds
            %   the obstacle Ziegler collision circle centers to the OCP
            %   Inputs:
            %   x_obs_c: double, obstacle vehicle CG X coordinate
            %   y_obs_c: double, obstacle vehicle CG Y coordinate
            %   psi_obs_c: double, obstacle vehicle CG yaw angle

            Defaults = {10,3,0};
            Defaults(1:nargin-1) = varargin;
            x_obs_c = Defaults{1};
            y_obs_c = Defaults{2};
            psi_obs_c = Defaults{3};

            % the obstacle is denoted a,b,c for its three circle centers
            obsLen = 4; % obstacle vehicle length
            obj.x_a = x_obs_c - obsLen/2*cos(psi_obs_c); 
            obj.y_a = y_obs_c - obsLen/2*sin(psi_obs_c);
            obj.x_b = x_obs_c;
            obj.y_b = y_obs_c;
            obj.x_c = x_obs_c + obsLen/2*cos(psi_obs_c); 
            obj.y_c = y_obs_c + obsLen/2*sin(psi_obs_c);
        end

        function add_dynamic_obstacle_centers(obj,x_traj,y_traj,psi_traj)
            %add_dynamic_obstacle_centers Add a dynamic obstacle collision
            %centers
            %   add_dynamic_obstacle_centers(obj) adds the obstacle Ziegler
            %   collision circle centers to the OCP using provided obstacle
            %   CG and yaw angle trajectories.
            %   Inputs:
            %   x_traj: 1x(obj.N+1) double, global x coordinate trajectory
            %   y_traj: 1x(obj.N+1) double, global x coordinate trajectory
            %   psi_traj: 1x(obj.N+1) double, yaw angle trajectory

            % apply the pre-defined CG and yaw angle trajectories
            obj.Xobs = [x_traj; y_traj; psi_traj];
            obj.px_obs = obj.Xobs(1,:);
            obj.py_obs = obj.Xobs(2,:);
            obj.psi_obs = obj.Xobs(3,:);

            obj.x_a = obj.px_obs - obj.b.*cos(obj.psi_obs);
            obj.y_a = obj.py_obs - obj.b.*sin(obj.psi_obs);
            obj.x_b = obj.px_obs;
            obj.y_b = obj.py_obs;
            obj.x_c = obj.px_obs + obj.a.*cos(obj.psi_obs);
            obj.y_c = obj.py_obs + obj.a.*sin(obj.psi_obs);
        end

        function add_collision_constraint(obj,varargin)
            %add_collision_constraint Add the collision constraint to the
            %OCP
            %   add_collision_constraint adds the collision constraint
            %   based on the ego and the obstacle collision centers.
            %   Inputs:
            %   R_collision: double, collision radius between any two circle
            %   centers from different objects.

            Defaults = {obj.R_col};
            Defaults(1:nargin-1) = varargin;
            R_collision = Defaults{1};
            R_sq = R_collision^2;

            obj.opti.subject_to((obj.x_1-obj.x_a).^2+(obj.y_1-obj.y_a).^2 > R_sq); % distance 1 to a
            obj.opti.subject_to((obj.x_1-obj.x_b).^2+(obj.y_1-obj.y_b).^2 > R_sq); % distance 1 to b
            obj.opti.subject_to((obj.x_1-obj.x_c).^2+(obj.y_1-obj.y_c).^2 > R_sq); % distance 1 to c
            obj.opti.subject_to((obj.x_2-obj.x_a).^2+(obj.y_2-obj.y_a).^2 > R_sq); % distance 2 to a
            obj.opti.subject_to((obj.x_2-obj.x_b).^2+(obj.y_2-obj.y_b).^2 > R_sq); % distance 2 to b
            obj.opti.subject_to((obj.x_2-obj.x_c).^2+(obj.y_2-obj.y_c).^2 > R_sq); % distance 2 to c
            obj.opti.subject_to((obj.x_3-obj.x_a).^2+(obj.y_3-obj.y_a).^2 > R_sq); % distance 3 to a
            obj.opti.subject_to((obj.x_3-obj.x_b).^2+(obj.y_3-obj.y_b).^2 > R_sq); % distance 3 to b
            obj.opti.subject_to((obj.x_3-obj.x_c).^2+(obj.y_3-obj.y_c).^2 > R_sq); % distance 3 to c
        end

        function set_solver_init(obj)
            %set_solver_init Set solver initial values
            %   set_solver_init(obj) sets the solver initial values for the
            %   set of decision variables selected

            obj.opti.set_initial(obj.vx, 1);
            obj.opti.set_initial(obj.vy, 1);
            obj.opti.set_initial(obj.px, 0);
            obj.opti.set_initial(obj.py, 0);
            obj.opti.set_initial(obj.psi, 0);
        end

        function warm_start_init(obj,sol)
            %warm_start_init Warm-start the initial condition
            % warm_start_init(obj,sol) warm-starts the intial condition by
            % using all the optimization variables from a previous solution

            obj.opti.set_initial(sol.value_variables());
        end

        function sol = solve(obj)
            %solve Solve the OCP and produce the solution object
            obj.opti.solver('ipopt'); % set numerical backend
            sol = obj.opti.solve();   % actual solve
        end

        function sol = solveSQP(obj)
            %solveSQP Solve the OCP with SQP solver
            opts = struct;
            opts.qpsol = 'qrqp';
            opts.qpsol_options.print_iter = true;
            opts.qpsol_options.error_on_fail = false;
            opts.print_status = true;
            opts.print_iteration = false;
            opts.print_time = false;
            obj.opti.solver('sqpmethod',opts);
            sol = obj.opti.solve();   % actual solve

        end
    end

    methods % methods related to the obstacle trajectory
        function [x_traj,y_traj,psi_traj] = make_const_v_trajectory(obj,X_init,vx,vy,r)
            %make_const_v_trajectory Make a constant velocity trajectory
            %   [x_traj,y_traj,psi_traj] =
            %   make_const_v_trajectory(Xobs_init,vx,vy,r) makes a constant
            %   velocity trajectory given the intial X_init = [x0,y0,psi0]
            %   and the specified velocities vx,vy,r.
            %   Inputs:
            %   X_init: 1x3 double, the initial [x0,y0,psi0] state of a
            %   moving obstacle
            %   vx: double, (constant) global x velocity
            %   vy: double, (constant) global y velocity
            %   r: double, (constant) yaw rate
            %   Outputs:
            %   x_traj: 1x(obj.N+1) double, global x trajectory
            %   y_traj: 1x(obj.N+1) double, global y trajectory
            %   psi_traj: 1x(obj.N+1) double, psi trajectory

            x_traj = X_init(1) + linspace(0,obj.T,obj.N+1)*vx;
            y_traj = X_init(2) + linspace(0,obj.T,obj.N+1)*vy;
            psi_traj = X_init(3) + linspace(0,obj.T,obj.N+1)*r;

        end
    end

    methods (Static) % vehicle dynamics
        function dx = vehdyn(x,u)
            %vehdyn Calculates the vehicle dynamics in state-space form
            % dx/dt = vehdyn(x,u)
            m = 1700;
            dx1 = u(1)/m;
            dx2 = u(2)/m;
            dx3 = x(1).*cos(x(5))-x(2).*sin(x(5));
            dx4 = x(1).*sin(x(5))+x(2).*cos(x(5));
            dx5 = (x(1).*u(2)-x(2).*u(1))./m./(x(1).^2+x(2).^2)-u(3); 
            dx = vertcat(dx1,dx2,dx3,dx4,dx5);
        end
    end
end