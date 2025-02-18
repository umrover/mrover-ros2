% right invariant EKF

classdef InvariantEKF < handle
    properties
        X
        P
        A
       
    end

    methods(Static)

        % put vector into Lie algebra basis
        function xhat = wedge(x)
            xhat = [0 -x(3) x(2) x(4) x(7);
                    x(3) 0 -x(1) x(5) x(8);
                    -x(2) x(1) 0 x(6) x(9);
                    0 0 0 0 0;
                    0 0 0 0 0];
        end

        % skew symmetric matrix of v
        function result = sup_x(v)
            result = [0 -v(3) v(2);
                   v(3) 0 -v(1);
                   -v(2) v(1) 0];
        end
    end

    methods
        % I-EKF initial values
        function obj = InvariantEKF()

            g_skew = InvariantEKF.sup_x([0; 0; -9.81]);

            rot_init = [0, -1, 0;
                        1, 0, 0;
                        0 0 1];

            % obj.X = eye(5);
            tens = [0; 0; 0];
            obj.X = [rot_init, zeros(3,1), tens;
                     zeros(1,3) 1 0;
                     zeros(1,3) 0 1];
            
            obj.A = [zeros(3), zeros(3), zeros(3);
                     g_skew, zeros(3), zeros(3);
                     zeros(3), eye(3), zeros(3)];


            obj.P = eye(9);

        end

        % adjoint
        function Ad_X = adjoint_X(obj)
            v_skew = [0 -obj.X(3,4) obj.X(2,4); obj.X(3,4) 0 -obj.X(1,4); -obj.X(2,4) obj.X(1,4) 0];
            p_skew = [0 -obj.X(3,5) obj.X(2,5); obj.X(3,5) 0 -obj.X(1,5); -obj.X(2,5) obj.X(1,5) 0];

            Ad_X = [obj.X(1:3,1:3), zeros(3), zeros(3);
                    v_skew * obj.X(1:3,1:3), obj.X(1:3,1:3), zeros(3);
                    p_skew * obj.X(1:3,1:3), zeros(3), obj.X(1:3,1:3)];
        end

        % w is 3x1 matrix of angular velocities (rad/s)
        % q is the 3x3 covariance matrix
        function gyro_predict(obj, w, q, dt)

            w_skew = [0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0];
            Ad_X = obj.adjoint_X();
            Q = [abs(q), zeros(3,3), zeros(3,3); % reference Invariant Kalman Filtering lecture II
                 zeros(3,3), zeros(3,3), zeros(3,3);
                 zeros(3,3), zeros(3,3), zeros(3,3)];
            Q_d = expm(obj.A * dt) * Q * dt * (expm(obj.A * dt)');

            % propogate
            obj.X(1:3,1:3) = obj.X(1:3,1:3) * expm(w_skew * dt);
            obj.P = expm(obj.A * dt) * obj.P * (expm(obj.A * dt))' + Ad_X * Q_d * Ad_X';
            

        end

        % a is 3x1 matrix of linear acceleration
        % q is the 3x3 covariance matrix
        function accel_predict(obj, a, q, dt)

            g = [0; 0; -9.81];
            Ad_X = obj.adjoint_X();

            Q = [zeros(3,3), zeros(3,3), zeros(3,3); % reference Invariant Kalman Filtering lecture II
                 zeros(3,3), abs(q), zeros(3,3);
                 zeros(3,3), zeros(3,3), abs(q) * dt];
            Q_d = expm(obj.A * dt) * Q * dt * (expm(obj.A * dt)');


            % propogate
            temp = obj.X;
            temp(1:3,4) = obj.X(1:3,4) + obj.X(1:3,1:3) * a * dt - g * dt; % velocity
            temp(1:3,5) = obj.X(1:3,5) + obj.X(1:3,4) * dt + 0.5 * obj.X(1:3,1:3) * a * dt.^2 - 0.5 * g * dt.^2; % position
            obj.X = temp;
            obj.P = expm(obj.A * dt) * obj.P * (expm(obj.A * dt)') + Ad_X * Q_d * Ad_X';

        end
        
        % p is a 3x1 matrix of cartesian position
        % V is the 3x1 noise
        function position_update(obj, p, V)

            H = [zeros(3), zeros(3), -eye(3)];

            Y = [-obj.X(1:3,1:3)' * p; 0; 1];
            b = [zeros(3,1); 0; 1];
            innov = obj.X * Y - b; % predicted - actual
            
            N = obj.X(1:3,1:3) * diag([abs(V(1)), abs(V(2)), abs(V(3))]) * obj.X(1:3,1:3)';
            S = H * obj.P * H' + N;
            L = obj.P * H' / S;

            delta = InvariantEKF.wedge(L * innov(1:3,1));

            % correction
            obj.X = expm(delta) * obj.X;
            obj.P = (eye(9) - L * H) * obj.P * (eye(9) - L * H)' + L * N * L';

        end



        % m is mag heading measurement in radians
        % V is the 10x1 noise in b
        function mag_update(obj, m, cov_m)

            % reference vector (predicted measurement)
            mag_ref = [1; 0; 0];
            H = [-InvariantEKF.sup_x(mag_ref), zeros(3), zeros(3)];

            Y = [-m / norm(m); 0; 0];
            b = [-mag_ref; 0; 0];
            innov = obj.X * Y - b; % predicted - actual

            N = obj.X(1:3,1:3) * cov_m * obj.X(1:3,1:3)';
            S = H * obj.P * H' + N;
            L = obj.P * H' / S;

            delta = InvariantEKF.wedge(L * innov(1:3,1));
            
            % correction
            obj.X = expm(delta) * obj.X;
            obj.P = (eye(9) - L * H) * obj.P * (eye(9) - L * H)' + L * N * L';

        end

        function accel_update(obj, a, cov_a)

            % reference vector (predicted measurement)
            accel_ref = [0; 0; -1];
            H = [-InvariantEKF.sup_x(accel_ref), zeros(3), zeros(3)];

            Y = [-a / norm(a); 0; 0];
            b = [-accel_ref; 0; 0];
            innov = obj.X * Y - b; % predicted - actual

            N = obj.X(1:3,1:3) * cov_a * obj.X(1:3,1:3)';
            S = H * obj.P * H' + N;
            L = obj.P * H' / S;

            delta = InvariantEKF.wedge(L * innov(1:3,1));
            
            % correction
            obj.X = expm(delta) * obj.X;
            obj.P = (eye(9) - L * H) * obj.P * (eye(9) - L * H)' + L * N * L';
            

        end

    end

end