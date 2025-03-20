% right invariant EKF

classdef InvariantEKF < handle
    properties
        X
        P
        A
       
    end

    methods(Static)

        % lift x (6x1) into SE(3) lie algebra
        function result = lift(x)
            result = [0 -x(3) x(2) x(4);
                      x(3) 0 -x(1) x(5);
                      -x(2) x(1) 0 x(6);
                      0 0 0 0];

        end

        % skew symmetric matrix of v (3x1)
        function result = sup_x(v)
            result = [0 -v(3) v(2);
                   v(3) 0 -v(1);
                   -v(2) v(1) 0];
        end
    end

    methods
        % InEKF initial values
        function obj = InvariantEKF()
            rot_init = [0, -1, 0;
                        1, 0, 0;
                        0 0 1];

            
            tens = [0; 0; 0];
            % obj.X = [rot_init, zeros(3,1), tens;
            %          zeros(1,3) 1 0;
            %          zeros(1,3) 0 1];
            obj.X = eye(4);
            
            obj.P = eye(6);
            obj.A = zeros(6);

        end

        % adjoint of current state
        function result = adj(obj)
            p_skew = obj.sup_x(obj.X(1:3,4));

            result = [obj.X(1:3,1:3), zeros(3);
                      p_skew * obj.X(1:3,1:3), obj.X(1:3,1:3)];
        end

        % angular velocity w (3x1)
        % angular velocity covariance cov_w (3x3)
        function gyro_predict(obj, w, cov_w, dt)

            Ad_X = obj.adj();
            Q = [abs(cov_w), zeros(3,3); % reference Invariant Kalman Filtering lecture II
                 zeros(3,3), zeros(3,3)];
            Q_d = expm(obj.A * dt) * Q * dt * (expm(obj.A * dt)');

            % propagate
            obj.X(1:3,1:3) = obj.X(1:3,1:3) * expm(obj.sup_x(w) * dt);
            obj.P = expm(obj.A * dt) * obj.P * (expm(obj.A * dt))' + Ad_X * Q_d * Ad_X';
            

        end

        % global frame velocity v (3x1)
        % global frame velocity noise n (3x1)
        function vel_predict(obj, v, n, dt)

            obj.A = [zeros(3), zeros(3);
                 obj.sup_x(v), zeros(3)];

            Ad_X = obj.adj();
            Q = [zeros(3,3), zeros(3,3);
                 zeros(3,3), diag([abs(n(1)), abs(n(2)), abs(n(3))])];
            Q_d = expm(obj.A * dt) * Q * dt * (expm(obj.A * dt)');

            % propagate
            obj.X(1:3,4) = obj.X(1:3,4) + v * dt;
            obj.P = expm(obj.A * dt) * obj.P * (expm(obj.A * dt))' + Ad_X * Q_d * Ad_X';

        end
        
        % global frame position p (3x1)
        % global frame position noise (3x1)
        function position_update(obj, p, n)

            H = [zeros(3), -eye(3)];

            Y = [-obj.X(1:3,1:3)' * p; 1];
            b = [zeros(3,1); 1];
            innov = obj.X * Y - b; % predicted - actual
            
            N = diag([abs(n(1)), abs(n(2)), abs(n(3))]);
            S = H * obj.P * H' + N;
            L = obj.P * H' / S;

            delta = InvariantEKF.lift(L * innov(1:3,1));

            % correction
            obj.X = expm(delta) * obj.X;
            obj.P = (eye(6) - L * H) * obj.P * (eye(6) - L * H)' + L * N * L';

        end



        % mag m (3x1)
        % mag covariance (3x3)
        function mag_update(obj, m, cov_m)

            % reference vector (predicted measurement)
            mag_ref = [1; 0; 0];
            H = [-InvariantEKF.sup_x(mag_ref), zeros(3)];

            Y = [-m / norm(m); 0];
            b = [-mag_ref; 0];
            innov = obj.X * Y - b; % predicted - actual

            N = obj.X(1:3,1:3) * cov_m * obj.X(1:3,1:3)';
            S = H * obj.P * H' + N;
            L = obj.P * H' / S;

            delta = InvariantEKF.lift(L * innov(1:3,1));
            
            % correction
            obj.X = expm(delta) * obj.X;
            obj.P = (eye(6) - L * H) * obj.P * (eye(6) - L * H)' + L * N * L';

        end

        % acceleration a (3x1)
        % acceleration covariance cov_a (3x1)
        function accel_update(obj, a, cov_a)

            % reference vector (predicted measurement)
            accel_ref = [0; 0; -1];
            H = [-InvariantEKF.sup_x(accel_ref), zeros(3)];

            Y = [-a / norm(a); 0];
            b = [-accel_ref; 0];
            innov = obj.X * Y - b; % predicted - actual

            N = obj.X(1:3,1:3) * cov_a * obj.X(1:3,1:3)';
            S = H * obj.P * H' + N;
            L = obj.P * H' / S;

            delta = InvariantEKF.lift(L * innov(1:3,1));
            
            % correction
            obj.X = expm(delta) * obj.X;
            obj.P = (eye(6) - L * H) * obj.P * (eye(6) - L * H)' + L * N * L';
            

        end

    end

end