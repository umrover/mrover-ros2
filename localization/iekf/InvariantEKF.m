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

            g_skew = [0 -9.81 0; 9.81 0 0; 0 0 0];

            rot_init = [-0.92671502096, 0.37576491309, 0;
                        -0.37576491309, -0.92671502096, 0;
                        0 0 1];

            % obj.X = eye(5);
            tens = [-10; 10; 10];
            % obj.X = [rot_init, zeros(3,1), tens;
                     % zeros(1,3) 1 0;
                     % zeros(1,3) 0 1];
            obj.X = eye(5);


            
            obj.A = [zeros(3), zeros(3), zeros(3);
                     g_skew, zeros(3), zeros(3);
                     zeros(3), eye(3), zeros(3)];


            obj.P = eye(9); % may need to set this differently

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
            Q_d = expm(obj.A * dt) * Q * dt * (expm(obj.A * dt)');

            disp("gyro Q_d:");
            disp(Q_d);
            disp("exp(A):");
            disp(expm(obj.A * dt));
            disp("expm(obj.A * dt) * P * expm(obj.A * dt)':");
            disp(expm(obj.A * dt) * obj.P * (expm(obj.A * dt)'));
            disp("Ad_X * Q_d * Ad_X':");
            disp(Ad_X * Q_d * Ad_X');


            % propogate
            obj.X(1:3,1:3) = obj.X(1:3,1:3) * expm(w_skew * dt);
            obj.P = expm(obj.A * dt) * obj.P * (expm(obj.A * dt))' + Ad_X * Q_d * Ad_X';

        end

        % a is 3x1 matrix of linear acceleration
        % q is the 3x3 covariance matrix
        function accel_predict(obj, a, q, dt)

            g = [0; 0; 9.81];
            Ad_X = obj.adjoint_X();

            % what Q should actually be, but will only work if rotation
            % drift is fixed
            % Q = [zeros(3,3), zeros(3,3), zeros(3,3);
            %      zeros(3,3), obj.X(1:3,1:3) * abs(q), zeros(3,3);
            %      zeros(3,3), zeros(3,3), obj.X(1:3,1:3) * abs(q) * dt];

            Q = [zeros(3,3), zeros(3,3), zeros(3,3);
                 zeros(3,3), abs(q), zeros(3,3);
                 zeros(3,3), zeros(3,3), abs(q) * dt];
            Q_d = expm(obj.A * dt) * Q * dt * (expm(obj.A * dt)'); % TODO: check during testing

            disp("accel Q_d:");
            disp(Q_d);
            
            
            % propogate
            temp = obj.X;
            temp(1:3,4) = obj.X(1:3,4) + obj.X(1:3,1:3) * a * dt + g * dt; % velocity
            temp(1:3,5) = obj.X(1:3,5) + obj.X(1:3,4) * dt + 0.5 * obj.X(1:3,1:3) * a * dt.^2 + 0.5 * g * dt.^2; % position

            disp("temp:");
            disp(temp);
            disp("P before:");
            disp(obj.P);


            obj.X = temp;
            disp("expm(obj.A * dt) * P * expm(obj.A * dt)':");
            disp(expm(obj.A * dt) * obj.P * (expm(obj.A * dt)'));
            disp("expm(A)");
            disp(expm(obj.A * dt));
            % disp("I + obj.A approx:");
            % disp(1000 * dt * (eye(9) + obj.A) * obj.P * 1000 * dt * (eye(9) + obj.A)');
            disp("Ad_X * Q_d * Ad_X':");
            disp(Ad_X * Q_d * Ad_X');
           
            
            obj.P = expm(obj.A * dt) * obj.P * (expm(obj.A * dt)') + Ad_X * Q_d * Ad_X';
            % obj.P = dt * (eye(9) + obj.A) * obj.P * dt * (eye(9) + obj.A)' + Ad_X * Q_d * Ad_X';
            disp("P after:");
            disp(obj.P);

        end
        
        % p is a 3x1 matrix of cartesian position
        % V is the 3x1 noise
        % V can be positive or negative, but when used for noise matrix it
        % should always be positive...
        function position_update(obj, p, V)
            % disp("t = " + t);
            H = [zeros(3), zeros(3), -eye(3)];

            % get V, p into the robot frame
            % V = -obj.X(1:3,1:3)' * V;
            p = -obj.X(1:3,1:3)' * p;

            % Y = [p; 0; 1] + [V; 0; 0];
            Y = [p; 0; 1];
            b = [zeros(3,1); 0; 1];
            innov = obj.X * Y - b;

            disp(innov);
            
            % N = obj.X(1:3,1:3) * diag([abs(V(1)), abs(V(2)), abs(V(3))]) * obj.X(1:3,1:3)';
            N = diag([abs(V(1)), abs(V(2)), abs(V(3))]);
            disp("HPH':");
            disp(H * obj.P * H');
            S = H * obj.P * H' + N;
            L = obj.P * H' / S; % L should ALWAYS be negative, otherwise something is wrong

            delta = InvariantEKF.wedge(L * innov(1:3,1));

            disp("P * H':");
            disp(obj.P * H');
            % disp(S);
            % disp(L);
            disp("delta:")
            disp(delta);
            
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

    end

end