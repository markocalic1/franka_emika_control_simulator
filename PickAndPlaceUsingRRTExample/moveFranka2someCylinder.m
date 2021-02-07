[franka,config,env] = franka_and_env;

close all

figure("Name","Pick and Place Using RRT",...
    "Units","normalized",...
    "OuterPosition",[0, 0, 1, 1],...
    "Visible","on");
show(franka,config,"Visuals","off","Collisions","on");
hold on

for i = 1:length(env)
    show(env{i});
end


planner = manipulatorRRT(franka, env);

planner.MaxConnectionDistance = 0.3;
planner.ValidationDistance = 0.1;

startConfig = config;

% Get number of joints and the end-effector RBT frame.
numJoints = numel(startConfig);
endEffector = "panda_hand";

%Specify the trajectory time step and approximate desired tool speed.
timeStep = 0.1; % seconds
toolSpeed = 0.1; % m/s


% Set the initial and final end-effector pose.
jointInit = startConfig;
taskInit = getTransform(franka,jointInit,endEffector);

%Move robot hand above Tcyl defined in franka_and_env.m only changed z
%variable 
taskFinal = trvec2tform([0.5, 0.15 ,0.4])*axang2tform([1 1 0 pi]);




distance = norm(tform2trvec(taskInit)-tform2trvec(taskFinal)); 
initTime = 0;
finalTime = (distance/toolSpeed) - initTime;
trajTimes = initTime:timeStep:finalTime;
timeInterval = [trajTimes(1); trajTimes(end)];

% generates a trajectory that interpolates between two 4-by-4 homogeneous transformations,
% T0 and TF, with points based on the time interval and given time samples.
[taskWaypoints,taskVelocities] = transformtraj(taskInit,taskFinal,timeInterval,trajTimes); 



% Create a inverse kinematics object for the robot.
ik = inverseKinematics('RigidBodyTree',franka);
ik.SolverParameters.AllowRandomRestart = false;
weights = [1 1 1 1 1 1];


% Calculate the initial and desired joint configurations using inverse kinematics.
initialGuess = wrapToPi(jointInit);
jointFinal = ik(endEffector,taskFinal,weights,initialGuess);
jointFinal = wrapToPi(jointFinal);


% plans a path between the specified start and goal configurations using the
% manipulator rapidly exploring random trees (RRT) planner rrt.
rng('default');
path = plan(planner,config,jointFinal);

interpStates = interpolate(planner, path);

for i = 1:2:size(interpStates,1)
    show(franka, interpStates(i,:),...
        "PreservePlot", false,...
        "Visuals","off",...
        "Collisions","on");
    title("Plan 1: MaxConnectionDistance = 0.3")
    drawnow;
end

