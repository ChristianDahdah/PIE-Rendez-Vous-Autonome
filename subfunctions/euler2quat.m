function quat = euler2quat(euler)

roll = euler(1);
pitch = euler(2);
yaw = euler(3);

cr = cos(roll * 0.5);
sr = sin(roll * 0.5);
cy = cos(yaw * 0.5);
sy = sin(yaw * 0.5);
cp = cos(pitch * 0.5);
sp = sin(pitch * 0.5);


qw = cr * cy * cp - sr * sy * sp;

qx = sr * cy * cp + cr * sy * sp;

qy = cr * sy * cp + sr * cy * sp;

qz = cr * cy * sp - sr * sy * cp;

%To verifie the order of qz and qy
quat = [qw qx qz qy];

end