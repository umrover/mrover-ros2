filter = QuatIEKF();

num_samples = 200;
rng("shuffle");

gyro_raw = zeros(num_samples, 1);
euler_dead_reckoning = zeros(num_samples, 3);
euler_filtered = zeros(num_samples, 3);

dt = 0.02;
t  = 0:dt:num_samples*dt-dt;


gyro_covariance = eye(3) * 0.1;
accel_covariance = eye(3) * 0.00001;





% dead reckoning run
for i = 1:num_samples

    

    orientation_q = quaternion(filter.g(1:4)');
    euler_angles = quat2eul(orientation_q);
    euler_dead_reckoning(i,:) = flip(euler_angles);

    a = -5;
    b = 5;

    gyro_noise_x = a + (b-a).*rand(1,1);
    gyro_noise_y = a + (b-a).*rand(1,1);
    gyro_noise_z = a + (b-a).*rand(1,1);

    filter.gyro_predict([0 + gyro_noise_x; 0 + gyro_noise_y; 0 + gyro_noise_z], [gyro_covariance(1,1); gyro_covariance(2,2); gyro_covariance(3,3)], [0; 0; 0], dt);
    % disp(filter.g);
    

end

filter = QuatIEKF();


% filter run
for i = 1:num_samples

    orientation_q = quaternion(filter.g(1:4)');
    euler_angles = quat2eul(orientation_q);
    euler_filtered(i,:) = flip(euler_angles);


    if (mod(i, 1) == 0)

    
        a = -5;
        b = 5;
    
        gyro_noise_x = a + (b-a).*rand(1,1);
        gyro_noise_y = a + (b-a).*rand(1,1);
        gyro_noise_z = a + (b-a).*rand(1,1);
    
        filter.gyro_predict([0 + gyro_noise_x; 0 + gyro_noise_y; 0 + gyro_noise_z], [gyro_covariance(1,1); gyro_covariance(2,2); gyro_covariance(3,3)], [0; 0; 0], dt);


    end

    

    a = -0.01;
    b = 0.01;

    accel_noise_x = a + (b-a).*rand(1,1);
    accel_noise_y = a + (b-a).*rand(1,1);
    accel_noise_z = a + (b-a).*rand(1,1);


    filter.accel_correct([0.0 + accel_noise_x; 0.0 + accel_noise_y; -1 + accel_noise_z], accel_covariance);

    disp(filter.g);
    disp(filter.P);
    
    
end


clf;
hold on;
plot(t, euler_dead_reckoning(:,1));
plot(t, euler_dead_reckoning(:,2));
% plot(t, euler_dead_reckoning(:,3));
plot(t, euler_filtered(:,1));
plot(t, euler_filtered(:,2));
% plot(t, euler_filtered(:,3));
xlabel("t");
ylabel("radians");
legend("dead reckoning roll", "dead reckoning pitch", "filtered roll", "filtered pitch");
hold off;




% disp(obj.g);
% disp(obj.P);

% quat = quaternion(filter.g(1:4)');
% disp(quat);
% poseplot(quat);

