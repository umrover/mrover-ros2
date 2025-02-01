% right invariant EKF for quaternion

classdef QuatIEKF < handle
    properties
        g
        g_err

        % q
        % B
        % 
        % q_err
        % B_err
        
        P
       
    end

    methods(Static)

        function result = quat_to_rotm(q)
            q0 = q(1);
            qv = q(2:end);

            q_v_x = [0 -qv(3) qv(2);
                     qv(3) 0 -qv(1);
                     -qv(2) qv(1) 0];

            result = (q0.^2 - qv' * qv) * eye(3) + 2 * qv * qv' - 2 * q0 * q_v_x;

        end

        % quat is a 1x4 array representing a quaternion (w x y z]
        function A = A(quat)

            q_0 = quat(1)';
            q_v = quat(2:end)';
            q_v_x = QuatIEKF.wedge(q_v);
            
            A = (q_0.^2 - q_v' * q_v) * eye(3) + 2 * (q_v * q_v') - 2 * q_0 * q_v_x;
        end

        function result = quat_multiply(q, p)
            % disp(q);
            % disp(p);

            q0 = q(1);
            qv = q(2:end);

            p0 = p(1);
            pv = p(2:end);

            % disp("qv");
            % disp(qv);
            % disp("pv");
            % disp(pv);

            scalar_part = q0 * p0 - qv' * pv;
            vector_part = q0 * pv + p0 * qv + cross(qv, pv);

            result = [scalar_part; vector_part];

        end



        function result = quat_conjugate(q)

            result = [q(1); -1 * q(2:end)];

        end

        % skew symmetric matrix of v
        function result = sup_x(v)
            result = [0 -v(3) v(2);
                   v(3) 0 -v(1);
                   -v(2) v(1) 0];
        end

        
        % see eq 4 in paper
        function result = exp_q(x)
            scalar_part = cos(norm(x));
            vector_part = (x / norm(x)) * sin(norm(x));

            result = [scalar_part; vector_part];
            

        end

        
    end


    methods
        % Q-IEKF initial values
        function obj = QuatIEKF()

            % obj.g = [1 0 0 0 0 0 0]';
            obj.g = [0.0050038, 0.07056, 0.07056, 0.9949962, 0, 0, 0]';

            % see eq 14 in paper
            obj.g_err = [1 0 0 0 0 0 0]';
            obj.P = 500 * eye(6);
           

        end

        % w is a 3x1 array of angular velocities (rad/s)
        % n is a 3x1 array of noise
        function gyro_predict(obj, w, n_v, n_u, dt)
            

            q = obj.g(1:4);
            B = obj.g(5:7);
            q_err = obj.g_err(1:4);
            B_err = obj.g_err(5:7);

            % see eq 21 and 22 of paper
            n_v_est = QuatIEKF.quat_multiply(QuatIEKF.quat_multiply(q, [0; n_v]), QuatIEKF.quat_conjugate(q));
            I_w_est = QuatIEKF.quat_multiply(QuatIEKF.quat_multiply(q, [0; w - B]), QuatIEKF.quat_conjugate(q));
            n_u_est = QuatIEKF.quat_multiply(QuatIEKF.quat_multiply(q, [0; n_u]), QuatIEKF.quat_conjugate(q));
            % disp("n_u:")
            % disp(n_u);
            % disp("I_w_est:");
            % disp(I_w_est);
            % disp("n_u_est");
            % disp(n_u_est);

      
            obj.g_err(1:4) = q_err + QuatIEKF.quat_multiply(-0.5 * q_err, [0; B_err]) * dt + QuatIEKF.quat_multiply(0.5 * n_v_est, q_err) * dt;
            obj.g_err(1:4) = quatnormalize(obj.g_err(1:4)')';
            obj.g_err(5:7) = B_err + cross(QuatIEKF.quat_to_rotm(q_err) * (I_w_est(2:end) - n_u_est(2:end)), B_err) * dt - QuatIEKF.quat_to_rotm(q_err) * n_u_est(2:end) * dt;


            % see eq 27 of paper
            F = [zeros(3), -eye(3);
                 zeros(3), QuatIEKF.sup_x(I_w_est(2:end))];
            G = [QuatIEKF.quat_to_rotm(q)', zeros(3);
                 zeros(3), -QuatIEKF.quat_to_rotm(q)'];
            Q = diag([n_v; n_u].^2);

            % disp("Q");
            % disp(Q);
            % 
            % disp("GQG:");
            % disp(G * (expm(F * dt) * Q * dt * expm(F * dt)') * G');


            % see https://www.youtube.com/watch?v=WHBgSqRPpu4, eq 33 of
            % paper
            obj.P = expm(F * dt) * obj.P * expm(F * dt)' + G * (expm(F * dt) * Q * dt * expm(F * dt)') * G';

            % see https://stackoverflow.com/questions/46908345/integrate-angular-velocity-as-quaternion-rotation
            obj.g(1:4) = QuatIEKF.quat_multiply(q, QuatIEKF.exp_q((w - B) * dt));
            obj.g(5:7) = B;

            % disp("propogation amount:");
            % disp(QuatIEKF.exp_q((w - B) * dt));

            % disp("quat after gyro:");
            % disp(obj.g(1:4));



           

            % obj.P = expm(F * dt) * obj.P * (expm(F * dt))' + expm(G * dt) * Q * (expm(G * dt))'; % TODO: check this error propogation
        end

        % takes in acceleration vector and associated covariance
        function accel_correct(obj, a, cov_a)

            % see https://matthewhampsey.github.io/blog/2020/07/18/mekf,
            % and eq 31, 34 of paper
            H = [QuatIEKF.sup_x([0; 0; -1]), zeros(3)];
            R = QuatIEKF.quat_to_rotm(obj.g(1:4))' * cov_a * QuatIEKF.quat_to_rotm(obj.g(1:4));

           
            % disp("R:");
            % disp(R);

            % disp("obj.P * H'");
            % disp(obj.P * H');
            % 
            % disp()

             % see eq 33 of paper
            K = obj.P * H' / (H * obj.P * H' + R);

            % disp("P:");
            % disp(obj.P);
            % 
            % disp("PH'");
            % disp(obj.P * H');
            % disp("H * obj.P");
            % disp(H * obj.P);
            % disp("HPH'");
            % disp(H * obj.P * H');
            % disp("HPH' + R");
            % disp(H * obj.P * H' + R);
            % disp("R:");
            % disp(R);
            % 
            % disp("K:");
            % disp(K);

            % innovation, see eq 29 of paper
            observation = QuatIEKF.quat_multiply(QuatIEKF.quat_multiply(obj.g(1:4), [0; a]), QuatIEKF.quat_conjugate(obj.g(1:4)));
            predicted = [0; 0; -1];
            innov = predicted - observation(2:end);

            % disp("predicted");
            % disp(predicted);
            % disp("observation:");
            % disp(observation);
            % disp("innovation");
            % disp(innov);
            

            % disp("innov:");
            % disp(innov);

            % see eq 17 of paper
            correction = K * innov;
            
            % disp("correction:")
            % disp(correction);

            

            % update, see eq 20 of paper
            obj.g(1:4) = QuatIEKF.quat_multiply(QuatIEKF.exp_q(-0.5 * correction(1:3)), obj.g(1:4));
            B_update = QuatIEKF.quat_multiply(QuatIEKF.quat_multiply(QuatIEKF.quat_conjugate(obj.g(1:4)), [0; correction(4:6)]), obj.g(1:4));
            % disp("B update:");
            % disp(B_update);
            % 
            obj.g(5:7) = obj.g(5:7) - B_update(2:end);

            % see eq 33 of paper
            obj.P = (eye(6) - K * H) * obj.P;

            % disp("P:");
            % disp(obj.P);


        end

        

        
        
        

      

    end

end