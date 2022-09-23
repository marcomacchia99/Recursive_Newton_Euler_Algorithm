function tau = NewtonEuler(Robot,gravity)

g = [0 0 0];
if gravity==1
    g(Robot.gaxis)=-9.81;
end

%joint position with respect to base frame
p = zeros(3,Robot.jnum);

%rotation matrix
R = zeros(3,3,Robot.jnum);

%center of mass
CoM = zeros(3,Robot.jnum);


for i=1:Robot.jnum
    %compute every center of mass
    CoM(:,i) = compute_CoM(p(:,i),Robot.len(i),Robot.q,i,Robot.jtypes);

    %compute positions of joints: since every CoM is in the exact middle of
    %the link, it is only necessary to multiply it by 2 (starting from the
    % previous origin point)to get to the origin position of the next joint
    if i~=Robot.jnum
        p(:,i+1) = p(:,i)+ 2*(CoM(:,i)-p(:,i));
    end

    %compute rotations and translations matrices
    R(:,:,i)=rot_mat(Robot.q(i),Robot.rot_axis(i),Robot.jtypes(i));
end

% Computing the rotation matrix of frame i wrt base frame,
% The matrix Rabs is used to take care of the orientation of the
% axis of frame i wrt to the base frame
for i=Robot.jnum:-1:2
    Rtemp=R(:,:,1);
    for j=2:i
        Rtemp = Rtemp*R(:,:,j);
    end
    for j=1:Robot.jnum
        Rtemp=Rtemp*Robot.Rabs(:,:,j);
    end
    R(:,:,i)=Rtemp;
    clearvars Rtemp
end



%% forward recursion

%position of joint i with respect to joint i-1
r = zeros(3,Robot.jnum);

%angular velocity of joint i with respect to 0
w = zeros (3,Robot.jnum);
w(:,1)=Robot.qdot(1)*R(:,3,1);

%velocity of joint i with respect to 0
v = zeros(3,Robot.jnum);

%angular acceleration of joint i with respect to 0
wdot = zeros (3,Robot.jnum);
wdot(:,1)=Robot.q2dot(1)*R(:,3,1);

%linear acceleration of joint i with respect to 0
vdot = zeros (3,Robot.jnum);

for i=2:Robot.jnum
    % Computing, using forward algorithm, r_i/i-1 and linear and angular
    % velocities and accelerations. 
    if Robot.jtypes(i)==0 %rotational
        r(:,i)=p(:,i)-p(:,i-1);

        w(:,i)=w(:,i-1)+R(:,3,i)*Robot.qdot(i);

        v(:,i)=v(:,i-1)+cross(w(:,i-1),r(:,i));

        wdot(:,i)=wdot(:,i-1)+cross(w(:,i-1),R(:,3,i))*Robot.qdot(i)+...
            R(:,3,i)*Robot.q2dot(i);

        vdot(:,i)=vdot(:,i-1)+cross(wdot(:,i-1),r(:,i))+...
            cross(w(:,i-1),cross(w(:,i-1),r(:,i)));

    else %prismatic
        r(:,i)=p(:,i)-p(:,i-1)+R(:,3,i)*Robot.q(i);

        w(:,i)=w(:,i-1);

        v(:,i)=v(:,i-1)+cross(w(:,i-1),r(:,i))+Robot.qdot(i)*R(:,3,i);

        wdot(:,i)=wdot(:,i-1);

        vdot(:,i)=vdot(:,i-1)+cross(wdot(:,i-1),r(:,i))+ ...
            cross(w(:,i-1),cross(w(:,i-1),r(:,i)))+ ...
            2*cross(w(:,i-1),R(:,3,i))*Robot.qdot(i)+R(:,3,i)*Robot.q2dot(i);
    end
end

%% backward recursion

% Dynamic force
D = zeros (3,Robot.jnum);

% Dynamic moment
delta = zeros(3,Robot.jnum);

% acceleartion of center of mass of link i wrt base frame
Vcdot = zeros(3,Robot.jnum);

% Force applied by joint i on joint i-1
F = zeros(3,Robot.jnum);

% Moment applied by joint i on joint i-1
M = zeros(3,Robot.jnum);

% Actuation torques/forces
tau  = zeros(1,Robot.jnum);

% Computing, using the backward algorithm, forces and moments of
% joint i wrt joint i-1 and the actuation toques/forces.
for i=Robot.jnum:-1:1

    Vcdot(:,i)= vdot(:,i)+cross(wdot(:,i),-(p(:,i)-CoM(:,i)))+ ...
    cross(w(:,i),cross(w(:,i),-(p(:,i)-CoM(:,i))));
    
    D(:,i)= Robot.mass(i)*Vcdot(:,i);
    
    delta(:,i) = R(:,:,i)*(Robot.I(:,:,i))*R(:,:,i)'*wdot(:,i)+...
        cross(w(:,i),R(:,:,i)*(Robot.I(:,:,i)*R(:,:,i)')*w(:,i));

    if i==Robot.jnum
        F(:,i)= -Robot.mass(i)*g' - Robot.Fext + D(:,i);
        M(:,i)= -Robot.Mext - cross(p(:,i)-CoM(:,i),F(:,i))+delta(:,i);
    else
        F(:,i)= F(:,i+1)-Robot.mass(i)*g' - Robot.Fext + D(:,i);
        M(:,i)= M(:,i+1)-Robot.Mext - cross((p(:,i)-CoM(:,i)),F(:,i))+ ...
            cross(r(:,i+1)+(p(:,i)-CoM(:,i)),F(:,i+1)) +delta(:,i);
    end

    if Robot.jtypes(i)==0 %rotational
        tau(i)= R(:,3,i)'*M(:,i);
    else %prismatic
        tau(i)= R(:,3,i)'*F(:,i);
    end
end
end