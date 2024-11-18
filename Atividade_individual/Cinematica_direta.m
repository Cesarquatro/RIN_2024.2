clc; clear; close all;
%% Ⅰ) Deininido parâmetros
l      = [166      , 135 ,250  , 160  , 72   , 106.5]; %[mm]
alpha  = [0        , pi/2, 0   , 0    , -pi/2, 0];
a      = [0        , 0   , l(3), l(4) , 0    , 0];
d      = [0        , pi/2, 0   , -pi/2, 0    , 0];
offset = [l(1)+l(2), 0   , 0   , 0    , l(3) , l(6)];

%% Ⅱ) Construção do Robo:
L = cell(1, length(l)); % Use uma célula ao invés de um vetor numérico

for i = 1:1:length(l)
    L{i} = RevoluteMDH('alpha', alpha(i), 'a', a(i), 'd', d(i), 'offset', offset(i));
end
L = [L{:}];

myrobot = SerialLink(L,'name','myrobot[5DOF]')
myrobot.teach('rpy/zyx')
q = L.getpos()