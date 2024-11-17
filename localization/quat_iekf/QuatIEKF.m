% right invariant EKF for quaternion

classdef QuatIEKF < handle
    properties
        q
        B
        P
       
    end

    methods(Static)
        % quat is a 1x4 array representing a quaternion (w x y z]
        function A = A(quat)

            q_0 = quat(1)';
            q_v = quat(2:end)';
            q_v_x = QuatIEKF.wedge(q_v);
            
            A = (q_0.^2 - q_v' * q_v) * eye(3) + 2 * (q_v * q_v') - 2 * q_0 * q_v_x;
        end

        % turn vector in corresponding quaternion representation (scalar
        % term = 0)
        function quat = to_quat(v)
            quat = quaternion([0 v]);
        end

        % TODO: put vector into multiplicative group form?? idk what this is called - the
        % superscript x thingy?
        function v_x = wedge(v)
            v_x = [0 -v(3) v(2);
                   v(3) 0 -v(1);
                   -v(2) v(1) 0];
        end

        % this is exp subscript p in the paper
        % TODO: how to incorporate divide by 2?
        function exp_p = exp_p(rotation_axis_vec)

            q_0 = cos(norm(rotation_axis_vec));
            q_v = (rotation_axis_vec / norm(rotation_axis_vec) * sin(norm(rotation_axis_vec)));

            % TODO: kind of a bandage solution
            q_v(isnan(q_v)) = 0;
            
            exp_p = [q_0; q_v];

        end

        
    end


    methods
        % I-EKF initial values
        function obj = QuatIEKF()

            obj.q = quaternion(1,0,0,0);
            obj.B = zeros(3,1);
            obj.P = eye(6);

        end

        % w is a 1x3 array of angular velocities (rad/s)
        % n is a 1x3 array of noise
        function propogate(obj, w, n, dt)
            
            % construct F, A, Q
            I_w = QuatIEKF.A(compact(obj.q))' * (w' - obj.B);
            N_v = diag(n);

            F = [zeros(3,3), -eye(3);
                 zeros(3,3), QuatIEKF.wedge(I_w)];
            G = [QuatIEKF.A(compact(obj.q))', zeros(3,3);
                 zeros(3,3), -QuatIEKF.A(compact(obj.q))'];
            Q = [N_v, zeros(3,3);
                 zeros(3,3), zeros(3,3)]; % TODO: for now assuming bias noise is 0
            
            % propogate
            obj.q = 0.5 * obj.q * exp(QuatIEKF.to_quat(((w' - obj.B) * dt)'));
            obj.P = expm(F * dt) * obj.P * (expm(F * dt))' + expm(G * dt) * Q * (expm(G * dt))'; % TODO: check this error propogation
        end

        % m is a 1x3 reference vector in world frame
        function correct(obj, m, n)
            % compute H, R, V
            H = [QuatIEKF.wedge(m) zeros(3,3)];
            % TODO: think this is supposed to be change in noise? For now
            % let's test with no noise
            R_1 = QuatIEKF.A(compact(obj.q))';
            R_2 = QuatIEKF.A(compact(obj.q));
            R = R_1 * diag(n) * R_2;

            %disp(R);

            V = QuatIEKF.A(compact(obj.q))' * n';

            % compute K, E, C
            K = obj.P * H' / (H * obj.P * H' + R);
            %disp(K);
            %disp(eye(3) + QuatIEKF.wedge([obj.P(1,1), obj.P(2,2), obj.P(3,3)]));
            % TODO: sus error
            %disp(QuatIEKF.wedge([obj.P(1,1), obj.P(2,2), obj.P(3,3)]) * m');
            errors = [obj.P(1,1); obj.P(2,2); obj.P(3,3); obj.P(4,4); obj.P(5,5); obj.P(6,6)];
            E = H * errors + V;

            disp(errors);
            disp(E);
            %E = m' - (QuatIEKF.wedge([obj.P(1,1), obj.P(2,2), obj.P(3,3)]) * m' + QuatIEKF.A(compact(obj.q))' * n');
            %disp(E);
            C = K * E;
            %disp(K);
            
            disp(C);
            disp(QuatIEKF.exp_p(C(1:3) / 2));
            temp = vertcat(0, C(1:3));
            disp(exp(quaternion(temp')));

            % correct
            obj.P = (eye(6) - K * H) * obj.P;
            
            obj.q = quaternion(QuatIEKF.exp_p(-C(1:3) / 2)') * obj.q;

            %obj.q = exp(-quaternion(QuatIEKF.to_quat(C(1:3)')) / 2) * obj.q;
            %obj.q = 0.5 * obj.q * exp(quaternion(QuatIEKF.to_quat(w)) * dt);
            
            %disp(obj.q' * quaternion(QuatIEKF.to_quat(C(4:end)')) * obj.q);
            % B_vec = compact(obj.q' * quaternion(QuatIEKF.to_quat(C(4:end)')) * obj.q);
            % obj.B = obj.B - B_vec(1,2:4)';

        end

        
        
        

      

    end

end