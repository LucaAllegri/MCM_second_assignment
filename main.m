%% Template Exam Modelling and Control of Manipulators
clc;
close all;
clear;
addpath('include'); % put relevant functions inside the /include folder 

%% Compute the geometric model for the given manipulator
iTj_0 = BuildTree();
disp('iTj_0')
disp(iTj_0);
jointType = [0 0 0 0 0 1 0]; % specify two possible link type: Rotational, Prismatic.
geometricModel = geometricModel(iTj_0,jointType);

%% Q1.3

%da b a e 
bTk = geometricModel.getTransformWrtBase(length(jointType));
disp('bTk')
disp(bTk);

T_0_2 = geometricModel.getTransformWrtBase(2);
% Compute the inverse of the transformation matrix T_0_2
T_0_6 = geometricModel.getTransformWrtBase(6);
T_0_2_inv = inv(T_0_2);
T_0_6_inv = inv(T_0_6);

first = T_0_6_inv  * T_0_2 ;
% second = inv(T_0_2_inv * T_0_6 ); secondo metodo 
disp('first')
disp(first);



%% Q1.4 Simulation
% Given the following configurations compute the Direct Geometry for the manipulator

% Compute iTj : transformation between the base of the joint <i>
% and its end-effector taking into account the actual rotation/traslation of the joint
qi = [pi/4, -pi/4, 0, -pi/4, 0, 0.15, pi/4];
geometricModel.updateDirectGeometry(qi)
disp('iTj')
disp(geometricModel.iTj);

% Compute the transformation of the ee w.r.t. the robot base
bTe = geometricModel.getTransformWrtBase(length(jointType));  
disp('bTe')
disp(bTe)

% Show simulation ?
show_simulation = true;

% Set initial and final joint positions
qf = [5*pi/12, -pi/4, 0, -pi/4, 0, 0.18, pi/5];

%Prove
%qf = [pi, 0, 0, 0, 0, 0,0];

%%%%%%%%%%%%% SIMULATION LOOP %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Simulation variables
% simulation time definition 
samples = 100;
t_start = 0.0;
t_end = 10.0;
dt = (t_end-t_start)/samples;
t = t_start:dt:t_end; 

pm = plotManipulators(show_simulation);
pm.initMotionPlot(t);

qSteps =[linspace(qi(1),qf(1),samples)', ...
    linspace(qi(2),qf(2),samples)', ...
    linspace(qi(3),qf(3),samples)', ...
    linspace(qi(4),qf(4),samples)', ...
    linspace(qi(5),qf(5),samples)', ...
    linspace(qi(6),qf(6),samples)', ...
    linspace(qi(7),qf(7),samples)'];

% LOOP 
for i = 1:samples

    brij= zeros(3,geometricModel.jointNumber);
    q = qSteps(i,1:geometricModel.jointNumber)';
    % Updating transformation matrices for the new configuration 
    geometricModel.updateDirectGeometry(q)
    % Get the transformation matrix from base to the tool
    bTe = geometricModel.getTransformWrtBase(length(jointType)); 

    %% ... Plot the motion of the robot 
    if (rem(i,0.1) == 0) % only every 0.1 sec
        for j=1:geometricModel.jointNumber
            bTi(:,:,j) = geometricModel.getTransformWrtBase(j);
        end
        pm.plotIter(bTi)
    end

end

pm.plotFinalConfig(bTi)

%% Q1.5
km = kinematicModel(geometricModel);
J6 = km.getJacobianOfLinkWrtBase(6);
disp("J6");
disp(J6);


%% Q1.6
km.updateJacobian();
disp("J");
disp(km.J);

%% Q1.7


q= [0.7, -0.1, 1, -1, 0, 0.03, 1.3]';
geometricModel.updateDirectGeometry(q);
km = kinematicModel(geometricModel);

km.updateJacobian();
dq = [0.9, 0.1, -0.2, 0.3, -0.8, 0.5, 0]';

V_0 = km.J*dq;

o_w_e0 = V_0(1:3);
o_v_e0 = V_0(4:6);

T_0_e = geometricModel.getTransformWrtBase(length(q));

R_0_e = T_0_e(1:3, 1:3);
r_0_e = T_0_e(1:3, 4);
r_0_e_skew = skew(r_0_e);

e_w_e0 = R_0_e'*o_w_e0;


e_v_e0 = (R_0_e' * r_0_e_skew' * o_w_e0) + (R_0_e' * o_v_e0);


disp('End-effector angular velocity :');
disp(e_w_e0);
disp('End-effector linear velocity:');
disp(e_v_e0);



