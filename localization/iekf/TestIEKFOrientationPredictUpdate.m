filter = InvariantEKF();

num_samples = 200;
rng("shuffle");

gyro_raw = zeros(num_samples, 1);
euler_dead_reckoning = zeros(num_samples, 3);
euler_filtered = zeros(num_samples, 3);

dt = 0.02;
t  = 0:dt:num_samples*dt-dt;


gyro_covariance = eye(3) * 0.1;
accel_covariance = eye(3) * 0.000001;
mag_covariance = eye(3) * 0.00001;



% dead reckoning run
for i = 1:num_samples

    orientation = filter.X(1:3,1:3);
    euler_angles = rotm2eul(orientation);
    euler_dead_reckoning(i,:) = flip(euler_angles);

    a = -5;
    b = 5;

    gyro_noise_x = a + (b-a).*rand(1,1);
    gyro_noise_y = a + (b-a).*rand(1,1);
    gyro_noise_z = a + (b-a).*rand(1,1);

    filter.gyro_predict([0 + gyro_noise_x; 0 + gyro_noise_y; 0 + gyro_noise_z], gyro_covariance, dt);
    % disp(filter.g);
    

end

filter = InvariantEKF();


% filter run
for i = 1:num_samples

    orientation = filter.X(1:3,1:3);
    euler_angles = rotm2eul(orientation);
    euler_filtered(i,:) = flip(euler_angles);


    if (mod(i, 1) == 0)


        a = -5;
        b = 5;

        gyro_noise_x = a + (b-a).*rand(1,1);
        gyro_noise_y = a + (b-a).*rand(1,1);
        gyro_noise_z = a + (b-a).*rand(1,1);

        filter.gyro_predict([0 + gyro_noise_x; 0 + gyro_noise_y; 0 + gyro_noise_z], gyro_covariance, dt);


    end



    a = -0.01;
    b = 0.01;

    accel_noise_x = a + (b-a).*rand(1,1);
    accel_noise_y = a + (b-a).*rand(1,1);
    accel_noise_z = a + (b-a).*rand(1,1);


    filter.accel_update([0.0 + accel_noise_x; 0 + accel_noise_y; -1 + accel_noise_z], accel_covariance);

    disp(filter.X);
    disp(filter.P);

    a = -0.01;
    b = 0.01;

    mag_noise_x = a + (b-a).*rand(1,1);
    mag_noise_y = a + (b-a).*rand(1,1);
    mag_noise_z = a + (b-a).*rand(1,1);


    filter.mag_update([1 + mag_noise_x; 0 + mag_noise_y; 0 + mag_noise_z], mag_covariance);

    disp(filter.X);
    disp(filter.P);



end

% disp(euler_dead_reckoning);
% disp(euler_filtered);

clf;
hold on;
plot(t, euler_dead_reckoning(:,1));
plot(t, euler_dead_reckoning(:,2));
plot(t, euler_dead_reckoning(:,3));
plot(t, euler_filtered(:,1));
plot(t, euler_filtered(:,2));
plot(t, euler_filtered(:,3));
xlabel("t");
ylabel("radians");
legend("dead reckoning roll", "dead reckoning pitch", "dead reckoning yaw", "filtered roll", "filtered pitch", "filtered yaw");
hold off;