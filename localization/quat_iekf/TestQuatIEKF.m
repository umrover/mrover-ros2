obj = QuatIEKF();

%poseplot(obj.Q);

obj.propogate([0, 0, 0.5], [0.1, 0.1, 0.1], 1);
% do not allow 0 vectors
%obj.correct([0 10 0], [0.01 0.01 0.01]);
obj.correct([0 0 0.1], [0.001 0.001 0.001]);
% obj.correct([0 0 0.1], [0.001 0.001 0.001]);
% obj.correct([0 0 0.1], [0.001 0.001 0.001]);
% obj.correct([0 0 0.1], [0.001 0.001 0.001]);
% obj.correct([0 0 0.1], [0.001 0.001 0.001]);
% obj.correct([0 0 0.1], [0.001 0.001 0.001]);
% obj.correct([0 0 0.5], [0 0 0]);
% obj.correct([0 0 0.5], [0 0 0]);
% obj.correct([0 0 0.5], [0 0 0]);
% obj.correct([0 0 0.5], [0 0 0]);
% obj.correct([0 0 0.5], [0 0 0]);
% obj.correct([0 0 0.5], [0 0 0]);
% obj.correct([0 0 0.5], [0 0 0]);
% obj.correct([0 0 0.5], [0 0 0]);
% obj.correct([0 0 0.5], [0 0 0]);
% obj.correct([0 0 0.5], [0 0 0]);
% obj.correct([0 0 0.5], [0 0 0]);
% obj.correct([0 0 0.5], [0 0 0]);
% obj.correct([0 0 0.5], [0 0 0]);
% obj.correct([0 0 0.5], [0 0 0]);
% obj.correct([0 0 0.5], [0 0 0]);
% obj.correct([0 0 0.5], [0 0 0]);
% obj.correct([0 0 0.5], [0 0 0]);
% obj.correct([0 0 0.5], [0 0 0]);
% obj.correct([0 0 0.5], [0 0 0]);
% obj.correct([0 0 0.5], [0 0 0]);
% obj.correct([0 0 0.5], [0 0 0]);
% obj.correct([0 0 0.5], [0 0 0]);
% obj.correct([0 0 0.5], [0 0 0]);
% obj.correct([0 0 0.5], [0 0 0]);
% obj.correct([0 0 0.5], [0 0 0]);
% obj.correct([0 0 0.5], [0 0 0]);
% obj.correct([0 0 0.5], [0 0 0]);

poseplot(obj.q);


%poseplot(quaternion([0 0 0 0.4]));