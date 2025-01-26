addpath('/MATLAB Drive/mrover-ros2/localization/iekf');

filter = InvariantEKF();

num_samples = 1500;
rng("shuffle");

reading_raw = zeros(num_samples, 1);
position_filtered = zeros(num_samples, 3);

dt = 0.02;
t  = 0:dt:num_samples*dt-dt;

% filter.position_update([5; 5; 5], [1.1; 1.1; 1.1]);
% disp(filter.X);
% disp(filter.P)
% filter.position_update([10; 10; 10], [0.1; 0.1; 0.1]);
% disp(filter.X);
% disp(filter.P)
% filter.position_update([5; 5; 5], [1.2; 1.2; 1.8]);
% disp(filter.X);
% disp(filter.P);
% filter.accel_predict([0; 0; -9.81], [0.2, 0, 0; 0, 0.2, 0; 0, 0, 0.2], dt);
% disp(filter.X);
% disp(filter.P);

covariance = eye(3);
%obj.accel_predict([0.0901; -0.2976; 30], covariance, 0.01);
% filter.gyro_predict([0.0; 2.0; 2.0], covariance, 0.2);
% disp(filter.X);
disp(filter.P);
disp(filter.X);

for i = 1:num_samples

    % if (mod(i, 4) == 0)
    %     filter.accel_predict([0; 0; -9.81],  [0.0001, 0, 0; 0, 0.0001, 0; 0, 0, 0.0001], dt);
    % end

    a = -10;
    b = 10;
    n = 3;

    x = a + (b-a).*rand(1,1);
    y = a + (b-a).*rand(1,1);
    % x = 0;
    % y = 0;
    % z = GetSonar();
    z = a + (b-a).*rand(1,1);

    V = [abs(x); abs(y); abs(z)];

    % a_new = -1;
    % b_new = 1;
    % 
    % V_noise_sim = a_new + (b_new-a_new).*rand(3,1); 
    % V = [-x; -y; -z] + V_noise_sim;

    p = [x; y; z];


    filter.position_update(p, V);

    reading_raw(i,:) = x;
    position_filtered(i,:) = filter.X(1:3,5)';

    disp("X:")
    disp(filter.X);
end

% disp(position_filtered(:,1))
scatter(t, reading_raw, 1);
hold on;
plot(t, position_filtered(:,1));
hold off;