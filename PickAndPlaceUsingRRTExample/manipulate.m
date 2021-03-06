[franka,config,env] = exampleHelperLoadPickAndPlaceRRT;

figure("Name","Pick and Place Using RRT",...
    "Units","normalized",...
    "OuterPosition",[0, 0, 1, 1],...
    "Visible","on");
show(franka,config,"Visuals","off","Collisions","on");
hold on


% for i = 1:length(env)
%     show(env{i});
% end

planner = manipulatorRRT(franka, env);

planner.MaxConnectionDistance = 0.3;
planner.ValidationDistance = 0.1;




% goalConfig = [0.2371   -0.0200    0.0542   -2.2272    0.0013    2.2072   -0.9670    0.0400    0.0400];



% This is the orientation offset between the end-effector in grasping pose and the can frame
% R = [0 0 1; 1 0 0; 0 1 0]; 
% targetPos = [-0.15,0.35,0.51];
% Tw_0 = trvec2tform(targetPos+[0,0,0.08]); 
% Te_w = rotm2tform(R);
% 
% Te_0ref = Tw_0*Te_w; % Reference end-effector pose in world coordinates, derived from WGR
% ss = ExampleHelperRigidBodyTreeStateSpace(franka);
% 
% ik = inverseKinematics('RigidBodyTree',franka);
% refGoalConfig = ik(ss.EndEffector,Te_0ref,ones(1,6),homeConfiguration(ss.RigidBodyTree));

% TGraspToCan = trvec2tform([[-0.15 0.52 0.46]])*axang2tform([1 0 0 -pi/2]);

viztree = interactiveRigidBodyTree(franka,"MarkerBodyName","panda_hand");
currConfig = homeConfiguration(viztree.RigidBodyTree);

viztree.MarkerBodyName = "panda_hand";
viztree.Configuration = currConfig;

fixedWaist = constraintPoseTarget("panda_hand");

raiseRightLeg = constraintPositionTarget("panda_hand","TargetPosition",[0.35 0 0.6]);
viztree.Constraints = { raiseRightLeg};       

addConfiguration(viztree)

rng('default');
path = plan(planner,startConfig,viztree.StoredConfigurations');

interpStates = interpolate(planner, path);

for i = 1:2:size(interpStates,1)
    show(franka, interpStates(i,:),...
        "PreservePlot", false,...
        "Visuals","off",...
        "Collisions","on");
    title("Plan 1: MaxConnectionDistance = 0.3")
    drawnow;
end




