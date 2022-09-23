%compute center of mass of a given link.
function CoM = compute_CoM(p,l,q,joint_num,joint_types)

angle = 0;

if(size(q,2)>2)
    for i=2:joint_num
        if joint_types(i)~=1
            angle = angle + q(i);
        end
    end
    CoM_x = p(1)+(l/2)*cos(angle)*cos(q(1));
    CoM_y = p(2)+(l/2)*cos(angle)*sin(q(1));
    CoM_z = p(3)+(l/2)*sin(angle);
    CoM = [CoM_x CoM_y CoM_z];
else
    for i=1:joint_num
        if joint_types(i)~=1
            angle = angle + q(i);
        end
    end
    CoM_x = p(1)+(l/2)*cos(angle);
    CoM_y = p(2)+(l/2)*sin(angle);
    CoM = [CoM_x CoM_y 0];
end

end