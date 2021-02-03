function [franka,config,env] = franka_and_env
% franka_and_env
% Helper function to load the Franka Emika Panda robot and a simple 
% environment of collision objects placed in the robot model world.
%
% This function is for example purposes and may be removed in a future
% release.
% Copyright 2020 The Mathworks, Inc. 

% The Franka Emika Panda's inertial elements are missing. For the purposes
% of motion planning (since dynamics is not involved), it is acceptable to ignore this warning.
warnState = warning("off", "robotics:robotmanip:urdfimporter:IntertialComponentsMissing");
cleanup = onCleanup(@()warning(warnState));
% Load the robot model
franka = loadrobot("frankaEmikaPanda", "DataFormat","row",'Gravity',[0 0 -9.81]);

% Open the gripper during planning
% franka.Bodies{10}.Joint.HomePosition = 0.04;
% franka.Bodies{11}.Joint.HomePosition = 0.04;
% franka.Bodies{10}.Joint.PositionLimits = [0.04, 0.04];
% franka.Bodies{11}.Joint.PositionLimits = [0.04, 0.04];

% Calling the checkCollision function on the robot can aid in finding which
% link-pairs of the robot are in collision.
% Notice that the robot is in self-collision in its home configuration because link 7 and link 11 collide with
% link 5, and link 9 is in collision with link 7. 
% [isColliding, sepdist, ~] = checkCollision(franka, homeConfiguration(franka), "Exhaustive", "on");
% [r, c] = find(isnan(sepdist));

% Modify the home configuration of the robot
config = homeConfiguration(franka);
config(7) = 0.65;
config(6) = 0.75;

% Replace the link 9's collision cylinder with a close approximation which
% isn't in collision with link 7
clearCollision(franka.Bodies{9});
addCollision(franka.Bodies{9}, "cylinder", [0.07, 0.05], trvec2tform([0.0, 0, 0.025]));

% Create environment as a set of collision objects.
bench = collisionBox(0.5, 0.9, 0.05);
belt1 = collisionBox(1.3, 0.4, 0.235);
barricade = collisionBox(0.8, 0.03, 0.35);

TBench = trvec2tform([0.35 0 0.2]);
TBelt1 = trvec2tform([0 -0.6 0.2]);

bench.Pose = TBench;
belt1.Pose = TBelt1;
barricade.Pose = trvec2tform([0.3, -0.25, 0.4]);
cylinder1 = collisionCylinder(0.03, 0.1);
cylinder2 = collisionCylinder(0.03, 0.1);
cylinder3 = collisionCylinder(0.03, 0.1);

TCyl = trvec2tform([0.5 0.15 0.278]);
TCyl2 = trvec2tform([0.52 0 0.278]);
TCyl3 = trvec2tform([0.4 -0.1 0.2758]);

cylinder1.Pose = TCyl;
cylinder2.Pose = TCyl2;
cylinder3.Pose = TCyl3;
env = {bench, belt1, cylinder1, cylinder2, cylinder3, barricade};

end