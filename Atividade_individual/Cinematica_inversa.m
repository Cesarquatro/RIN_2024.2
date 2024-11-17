T_tcp_0= myrobot.fkine([10*pi/180
30*pi/180]);  % aqui colocar o vector dos ângulos das juntas
T = T_tcp_0;
q0= [0 0];
qi = myrobot.ikunc(T,q0);
qi = qi*180/pi;
fprintf("Inverse Kinematics - IKUNC - Numerical by optimization \n");






disp(qi)
