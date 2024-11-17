clc; clear; close all;
%% 
alpha = 2;
a = 10;
d = 20;
offset = 0;

L(1) = RevoluteMDH('alpha', alpha, 'a', a, 'd', d, 'offset', offset);
myrobot = SerialLink(L,'name','myrobot')
myrobot.teach('rpy/zyx')