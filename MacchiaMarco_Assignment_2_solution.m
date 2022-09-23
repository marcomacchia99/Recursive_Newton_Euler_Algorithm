clc
clear

addpath("matlab_functions\");
run("RobotData.m");

% exercise number in rows
% effect of gravity in columns
% configuration in pages
tau = cell(3,2,2);

tau{1,1,1} = NewtonEuler(RobData2_1,0);
disp("Exercise 2 - configuration 1 without gravity:");
disp(tau{1,1,1});

tau{1,2,1} = NewtonEuler(RobData2_1,1);
disp("Exercise 2 - configuration 1 with gravity:");
disp(tau{1,2,1});

tau{1,1,2} = NewtonEuler(RobData2_2,0);
disp("Exercise 2 - configuration 2 without gravity:");
disp(tau{1,1,2});

tau{1,2,2} = NewtonEuler(RobData2_2,1);
disp("Exercise 2 - configuration 2 with gravity:");
disp(tau{1,2,2});

tau{2,1,1} = NewtonEuler(RobData3_1,0);
disp("Exercise 3 - configuration 1 without gravity:");
disp(tau{2,1,1});

tau{2,2,1} = NewtonEuler(RobData3_1,1);
disp("Exercise 3 - configuration 1 with gravity:");
disp(tau{2,2,1});
tau{2,1,2} = NewtonEuler(RobData3_2,0);
disp("Exercise 3 - configuration 2 without gravity:");
disp(tau{2,1,2});

tau{2,2,2} = NewtonEuler(RobData3_2,1);
disp("Exercise 3 - configuration 2 with gravity:");
disp(tau{2,2,2});

tau{3,1,1} = NewtonEuler(RobData4,0);
disp("Exercise 4 - without gravity:");
disp(tau{3,1,1});

tau{3,2,1} = NewtonEuler(RobData4,1);
disp("Exercise 4 - with gravity:");
disp(tau{3,2,1});

clearvars -except tau