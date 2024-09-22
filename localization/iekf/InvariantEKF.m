% right invariant EKF

classdef InvariantEKF < handle
    properties
        X
        P
        Q
        A
       
    end

    methods
        % I-EKF initial values
        function obj = InvariantEKF()

            g_skew = [0 9.81 0; -9.81 0 0; 0 0 0];

            obj.X = eye(5);
            
            obj.A = [zeros(3), zeros(3), zeros(3);
                     g_skew, zeros(3), zeros(3);
                     zeros(3), eye(3), zeros(3)];


            obj.P = zeros(9);

            obj.Q = eye(9);
        end

        % adjoint
        function Ad_X = adjoint_X(obj)
            v_skew = [0 -obj.X(3,4) obj.X(2,4); obj.X(3,4) 0 -obj.X(1,4); -obj.X(2,4) obj.X(1,4) 0];
            p_skew = [0 -obj.X(3,5) obj.X(2,5); obj.X(3,5) 0 -obj.X(1,5); -obj.X(2,5) obj.X(1,5) 0];

            Ad_X = [obj.X(1:3,1:3), zeros(3), zeros(3);
                    v_skew * obj.X(1:3,1:3), obj.X(1:3,1:3), zeros(3);
                    p_skew * obj.X(1:3,1:3), zeros(3), obj.X(1:3,1:3)];
        end
        
        
        % put vector into Lie algebra basis
        function xhat = wedge(obj, x)
            xhat = [0 -x(3) x(2) x(4) x(7);
                    x(3) 0 -x(1) x(5) x(8);
                    -x(2) x(1) 0 x(6) x(9);
                    0 0 0 0 0;
                    0 0 0 0 0];
        end

        % w is 3x1 matrix of angular velocities
        function gyro_predict(obj, w, dt)

            w_skew = [0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0];
            Ad_X = obj.adjoint_X();
            
            % TODO: does Q change??

            % propogate
            obj.X(1:3,1:3) = obj.X(1:3,1:3) * expm(w_skew * dt);
            obj.P = expm(obj.A * dt) * obj.P * (expm(obj.A * dt))' + Ad_X * obj.Q * Ad_X';

        end

        % a is 3x1 matrix of linear acceleration
        function accel_predict(obj, a, dt)
            
            g = [0; 0; 9.81];
            Ad_X = obj.adjoint_X();

            % propogate
            temp = obj.X;
            temp(1:3,4) = obj.X(1:3,4) + obj.X(1:3,1:3) * a * dt - g * dt;
            temp(1:3,5) = obj.X(1:3,5) + obj.X(1:3,4) * dt + 0.5 * obj.X(1:3,1:3) * a * dt.^2 - 0.5 * g * dt.^2;
            obj.P = expm(obj.A * dt) * obj.P * (expm(obj.A * dt))' + Ad_X * obj.Q * Ad_X';
            obj.X = temp;

        end
        
        % p is a 3x1 matrix of cartesian position, n is 3x1 matrix of
        % associated noise
        function position_update(obj, p, n)
            H = [zeros(3), zeros(3), -eye(3)];

            % get n, p into the robot frame
            n = -obj.X(1:3,1:3)' * n;
            p = -obj.X(1:3,1:3)' * p;

            N = obj.X(1:3,1:3) * blkdiag(n(1), n(2), n(3)) / obj.X(1:3,1:3);
            Y = [p; 0; 1];
            b = [zeros(3,1); 0; 1];
            innov = obj.X * Y - b;

            S = H * obj.P * H' + N;
            L = obj.P * H' / S;

            delta = obj.wedge(L * innov(1:3,1));
            
            % correction
            obj.X = expm(delta) * obj.X;
            obj.P = (eye(9) - L * H) * obj.P * (eye(9) - L * H)' + L * N * L';
            disp(obj.P);
        end
        
        % TODO
        function mag_update()

        end

        function rtk_heading_update()

        end

        function accel_update()

        end

        function velocity_predict()

        end

        function rtk_pitch_update()

        end

    end

end