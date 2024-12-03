% right invariant EKF

classdef InvariantEKF < handle
    properties
        X
        P

        % TODO: confirm A is correct, and A should be static
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
            Q = [q, zeros(3,3), zeros(3,3);
                 zeros(3,3), zeros(3,3), zeros(3,3);
                 zeros(3,3), zeros(3,3), zeros(3,3)];
            Q_d = expm(obj.A * dt) * Q * dt * (expm(obj.A * dt));


            % propogate
            obj.X(1:3,1:3) = obj.X(1:3,1:3) * expm(w_skew * dt);
            obj.P = expm(obj.A * dt) * obj.P * (expm(obj.A * dt))' + Ad_X * Q_d * Ad_X';

        end

        % a is 3x1 matrix of linear acceleration
        % q is the 3x3 covariance matrix
        function accel_predict(obj, a, q, dt)

            g = [0; 0; 9.81];
            Ad_X = obj.adjoint_X();
            Q = [zeros(3,3), q, zeros(3,3);
                 zeros(3,3), zeros(3,3), zeros(3,3);
                 zeros(3,3), zeros(3,3), zeros(3,3)];
            Q_d = expm(obj.A * dt) * Q * dt * (expm(obj.A * dt)); % TODO: check during testing

            % propogate
            temp = obj.X;
            temp(1:3,4) = obj.X(1:3,4) + obj.X(1:3,1:3) * a * dt - g * dt; % velocity
            temp(1:3,5) = obj.X(1:3,5) + obj.X(1:3,4) * dt + 0.5 * obj.X(1:3,1:3) * a * dt.^2 - 0.5 * g * dt.^2; % position

            obj.X = temp;
            obj.P = expm(obj.A * dt) * obj.P * (expm(obj.A * dt))' + Ad_X * Q_d * Ad_X';

        end
        
        % p is a 3x1 matrix of cartesian position
        % V is the 3x1 noise
        function position_update(obj, p, V)
            H = [zeros(3), zeros(3), -eye(3)];

            % get V, p into the robot frame
            V = -obj.X(1:3,1:3)' * V;
            p = -obj.X(1:3,1:3)' * p;

            Y = [p; 0; 1] + [V; 0; 0];
            b = [zeros(3,1); 0; 1];
            innov = obj.X * Y - b;
            
            N = obj.X(1:3,1:3) * blkdiag(V(1), V(2), V(3)) * obj.X(1:3,1:3)';
            S = H * obj.P * H' + N;
            L = obj.P * H' / S;

            delta = InvariantEKF.wedge(L * innov(1:3,1));
            
            % correction
            obj.X = expm(delta) * obj.X;
            obj.P = (eye(9) - L * H) * obj.P * (eye(9) - L * H)' + L * N * L';
            
        end
        

        % m is mag heading measurement in radians
        % V is the 10x1 noise in b
        function mag_update(obj, m, V)

            g_skew = [0 -9.81 0;
                      9.81 0 0;
                      0 0 0];

            m_skew = [0 1 -1;
                      -1 0 1;
                      1 -1 0];

            H = [g_skew zeros(3,3) zeros(3,3);
                 zeros(2,3) zeros(2,3) zeros(2,3);
                 m_skew zeros(3,3) zeros(3,3);
                 zeros(2,3) zeros(2,3) zeros(2,3)];
            
            % get mag measurement in the robot frame
            eulerXYZ = rotm2eul(obj.X(1:3,1:3));
            roll = eulerXYZ(3);
            pitch = eulerXYZ(2);

            R_m = eul2rotm([m, pitch, roll]);
            X_m = [R_m, obj.X(1:3,4), obj.X(1:3,5);
                    zeros(1,3), 1, 0;
                    zeros(1,3), 0, 1];

            b = [0; 0; -9.81; 0; 0; 1; 1; 1; 0; 0];
            Y = [inv(X_m) zeros(5,5);
                 zeros(5,5), inv(X_m)] * b + V;

            innov = [obj.X zeros(5,5);
                     zeros(5,5), obj.X] * Y - b;

            N = [obj.X zeros(5,5); zeros(5,5), obj.X] * blkdiag(V(1), V(2), V(3), V(4), V(5), V(6), V(7), V(8), V(9), V(10)) * [obj.X zeros(5,5); zeros(5,5), obj.X]';
            S = H * obj.P * H' + N;
            L = obj.P * H' / S;
            
            delta = InvariantEKF.wedge(L * innov);
            
            % correction
            obj.X = expm(delta) * obj.X;
            obj.P = (eye(9) - L * H) * obj.P * (eye(9) - L * H)' + L * N * L';
            
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