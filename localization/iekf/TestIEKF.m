obj = InvariantEKF();
obj.gyro_predict([0.0; 1.0; 2.0], 0.2);
obj.position_update([0.0; 0; 1.5], [0.2; 0.2; 0.2]);

R = eul2rotm([0.0 0 3.0]);
%disp(obj.X)

obj.mag_update(3.0);

%obj.position_update([0.0; -1.5; 1.5], [0.2; 0.2; 0.05]);

rotation = obj.X(1:3,1:3);
translation = obj.X(1:3,5);

translation = transpose(translation);

transformation = se3(rotation, translation);

%ax = plotTransforms(se3);
ax = plotTransforms(transformation);
xlabel("x");
ylabel("y");
zlabel("z");
