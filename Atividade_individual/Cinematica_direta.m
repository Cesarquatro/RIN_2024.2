clc; clear; close all;
% Carlos Augusto Fernandes Leitão        - 211270628
% Cesar Augusto Mendes Cordeiro da Silva - 211270121
% Guilherme Bueno Guidetti               - 211270601
% Lucas de Camargo Mainente              - 211270661
% Rafael Kenji Issaka                    - 201270072
% Prof. Dr. Maurício Becerra Vargas
% Robótica Industrial

%% Cinemática Direta!
%% Ⅰ) Difinição dos Parâmetros:
% Comprimentos
l      = [166      , 135 ,250  , 160  , 72   , 106.5]; %[mm]

% Entrada dos ângulos das Juntas em graus pelo usuário:
disp("Coloque os ângulos das juntas em graus:")
th1 = deg2rad(input("θ₁: "));
th2 = deg2rad(input("θ₂: "));
th3 = deg2rad(input("θ₃: "));
th4 = deg2rad(input("θ₄: "));
th5 = deg2rad(input("θ₅: "));

% Resto dos parâmetros
alpha  = [0        ,-pi/2     , 0   , 0         , pi/2, 0]; % α (i-1) [rad]
a      = [0        ,0         , l(3), l(4)      , 0  , 0];   % a (i-1) [mm]
thetas = [th1      ,th2-(pi/2), th3 , th4+(pi/2), th5, 0];    % θ (i) [rad]
d      = [l(1)+l(2),0         , 0   , 0         , l(5), l(6)]; % d (i) [mm]




offset = [l(1)+l(2), 0    , 0   , 0    , l(3) , l(6)]; %












%% Uma hora:
% L = cell(1, length(l)); % Use uma célula ao invés de um vetor numérico
% 
% for i = 1:1:length(l)
%     L{i} = RevoluteMDH('alpha', alpha(i), 'a', a(i), 'd', d(i), 'offset', offset(i));
% end
% L = [L{:}];
% 
% myrobot = SerialLink(L,'name','myrobot[5DOF]')
% myrobot.teach('rpy/zyx')
% q = L.getpos()