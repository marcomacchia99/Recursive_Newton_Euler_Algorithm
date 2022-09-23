%compute rotation matrix
function rotm = rot_mat(angle,rot_axis,joint_type)

if joint_type==1
    rotm=eye(3);
else
    if rot_axis ==1
    rotm = zeros(3,3);
    rotm(1,1) = 1;
    rotm(2,2) = cos(angle);
    rotm(3,3) = cos(angle);
    rotm(3,2) = sin(angle);
    rotm(2,3) = -sin(angle);

    elseif rot_axis==2
        rotm = zeros(3,3);
        rotm(2,2) = 1;
        rotm(1,1) = cos(angle);
        rotm(3,3) = cos(angle);
        rotm(3,1) = -sin(angle);
        rotm(1,3) = sin(angle);

    elseif rot_axis==3
        rotm = zeros(3,3);
        rotm(3,3) = 1;
        rotm(1,1) = cos(angle); 
        rotm(2,2) = cos(angle);
        rotm(1,2) = -sin(angle);
        rotm(2,1) = sin(angle);
    end


end