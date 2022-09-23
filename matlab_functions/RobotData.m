clc
clear

% axis: 1 - x, 2 - y, 3 - z
%joints: 0 - rotational, 1 - prismatic

RobData2_1 = struct();
RobData2_1.jnum = 2; % number of joints
RobData2_1.jtypes = [0 0]; % type of joints
RobData2_1.rot_axis = [3 3]; %rotation axis of the joint
RobData2_1.gaxis = 2; % gravity axis on which acts gravity
RobData2_1.mass = [22 19]; % mass of links
RobData2_1.len = [1 0.8]; % length of links
RobData2_1.I = zeros(3,3,RobData2_1.jnum);% inertia matrix
RobData2_1.I(:,:,1) = diag([0.4 0.4 0.4]);
RobData2_1.I(:,:,2) = diag([0.3 0.3 0.3]);
RobData2_1.q = [deg2rad(20) deg2rad(40)]; % configuration of the robot (positions of joints)
RobData2_1.qdot = [0.2 0.15]; % velocities of joints
RobData2_1.q2dot = [0.1 0.085]; % acceleration of joints
RobData2_1.Mext = [0; 0; 0]; % external moments acting on the robot
RobData2_1.Fext = [0; 0; 0]; % external forces acting on the robot
RobData2_1.Rabs = zeros(3,3,RobData2_1.jnum);
RobData2_1.Rabs(:,:,1) = eye(3);
RobData2_1.Rabs(:,:,2) = eye(3);

RobData2_2 = struct();
RobData2_2.jnum = 2;
RobData2_2.jtypes = [0 0]; 
RobData2_2.rot_axis = [3 3];
RobData2_2.gaxis = 2; 
RobData2_2.mass = [22 19]; 
RobData2_2.len = [1 0.8]; 
RobData2_2.I = [0.4 0.3]; 
RobData2_2.I = zeros(3,3,RobData2_2.jnum);
RobData2_2.I(:,:,1) = diag([0.4 0.4 0.4]);
RobData2_2.I(:,:,2) = diag([0.3 0.3 0.3]);
RobData2_2.q = [deg2rad(90) deg2rad(45)];
RobData2_2.qdot = [-0.8 0.35]; 
RobData2_2.q2dot = [-0.4 0.1]; 
RobData2_2.Mext = [0; 0; 0]; 
RobData2_2.Fext = [0; 0; 0];
RobData2_2.Rabs = zeros(3,3,RobData2_2.jnum);
RobData2_2.Rabs(:,:,1) = eye(3);
RobData2_2.Rabs(:,:,2) = eye(3);

RobData3_1 = struct();
RobData3_1.jnum = 2;
RobData3_1.jtypes = [0 1];
RobData3_1.rot_axis = [3 3];
RobData3_1.gaxis = 2;
RobData3_1.mass = [10 6];
RobData3_1.len = [1 0];
RobData3_1.I = zeros(3,3,RobData3_1.jnum);
RobData3_1.I(:,:,1) = diag([0.4 0.4 0.4]);
RobData3_1.I(:,:,2) = diag([0.3 0.3 0.3]);
RobData3_1.q = [deg2rad(20) 0.2];
RobData3_1.qdot = [0.08 0.03];
RobData3_1.q2dot = [0.1 0.01];
RobData3_1.Mext = [0; 0; 0];
RobData3_1.Fext = [0; 0; 0];
RobData3_1.Rabs = zeros(3,3,RobData3_1.jnum);
RobData3_1.Rabs(:,:,1) = eye(3);
RobData3_1.Rabs(:,:,2) = [0 0 1; 0 1 0;-1 0 0 ];

RobData3_2 = struct();
RobData3_2.jnum = 2;
RobData3_2.jtypes = [0 1];
RobData3_2.rot_axis = [3 3];
RobData3_2.gaxis = 2;
RobData3_2.mass = [10 6];
RobData3_2.len = [1 0];
RobData3_2.I = zeros(3,3,RobData3_2.jnum);
RobData3_2.I(:,:,1) = diag([0.4 0.4 0.4]);
RobData3_2.I(:,:,2) = diag([0.3 0.3 0.3]);
RobData3_2.q = [deg2rad(120) 0.6];
RobData3_2.qdot = [-0.4 -0.08];
RobData3_2.q2dot = [-0.1 -0.01];
RobData3_2.Mext = [0; 0; 0];
RobData3_2.Fext = [0; 0; 0];
RobData3_2.Rabs = zeros(3,3,RobData3_2.jnum);
RobData3_2.Rabs(:,:,1) = eye(3);
RobData3_2.Rabs(:,:,2) = [0 0 1; 0 1 0;-1 0 0 ];

RobData4 = struct();
RobData4.jnum = 3;
RobData4.jtypes = [0 0 0];
RobData4.rot_axis = [3 2 2];
RobData4.gaxis = 3;
RobData4.mass = [20 20 6];
RobData4.len = [1 0.8 0.35];
RobData4.I = zeros(3,3,RobData4.jnum);
RobData4.I(:,:,1) = diag([0.2 0.2 0.8]);
RobData4.I(:,:,2) = diag([0.2 0.2 0.8]);
RobData4.I(:,:,3) = diag([0.08 0.08 0.1]);
RobData4.q = [deg2rad(20) deg2rad(40) deg2rad(10)];
RobData4.qdot = [0.2 0.15 -0.2];
RobData4.q2dot = [0.1 0.085 0];
RobData4.Mext = [0; 0; 0];
RobData4.Fext = [0; 0; 0];
RobData4.Rabs = zeros(3,3,RobData4.jnum);
RobData4.Rabs(:,:,1) = eye(3);
RobData4.Rabs(:,:,2) = [1 0 0; 0 0 -1;0 1 0 ];
RobData4.Rabs(:,:,3) = eye(3);
