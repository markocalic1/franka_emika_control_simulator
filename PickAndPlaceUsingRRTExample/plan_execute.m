% The Franka Emika Panda's inertial elements are missing. For the purposes
% of motion planning (since dynamics is not involved), it is acceptable to ignore this warning.
warnState = warning("off", "robotics:robotmanip:urdfimporter:IntertialComponentsMissing");
cleanup = onCleanup(@()warning(warnState));
% Load the robot model
robot = loadrobot("frankaEmikaPanda", "DataFormat","row",'Gravity',[0 0 -9.81]);

currentRobotJConfig = homeConfiguration(robot);

numJoints = numel(currentRobotJConfig);
endEffector = "panda_hand";

timeStep = 0.1; % seconds
toolSpeed = 0.1; % m/s


jointInit = currentRobotJConfig;
taskInit = getTransform(robot,jointInit,endEffector);

% figure("Name","Pick and Place Using RRT",...
%     "Units","normalized",...
%     "OuterPosition",[0, 0, 1, 1],...
%     "Visible","on");
% show(franka,currentRobotJConfig,"Visuals","off","Collisions","on");
% hold on

taskFinal = trvec2tform([0.4,0,0.6])*axang2tform([0 1 0 pi]);

distance = norm(tform2trvec(taskInit)-tform2trvec(taskFinal));


initTime = 0;
finalTime = (distance/toolSpeed) - initTime;
trajTimes = initTime:timeStep:finalTime;
timeInterval = [trajTimes(1); trajTimes(end)];

[taskWaypoints,taskVelocities] = transformtraj(taskInit,taskFinal,timeInterval,trajTimes); 


ik = inverseKinematics('RigidBodyTree',robot);
ik.SolverParameters.AllowRandomRestart = false;
weights = [1 1 1 1 1 1];

initialGuess = wrapToPi(jointInit);
jointFinal = ik(endEffector,taskFinal,weights,initialGuess);
jointFinal = wrapToPi(jointFinal);

ctrlpoints = [jointInit',jointFinal'];
jointConfigArray = cubicpolytraj(ctrlpoints,timeInterval,trajTimes);
jointWaypoints = bsplinepolytraj(jointConfigArray,timeInterval,1);