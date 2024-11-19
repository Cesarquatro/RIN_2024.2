clc; clear; close all;
% Carlos Augusto Fernandes Leitão        - 211270628
% Cesar Augusto Mendes Cordeiro da Silva - 211270121
% Guilherme Bueno Guidetti               - 211270601
% Lucas de Camargo Mainente              - 211270661
% Rafael Kenji Issaka                    - 201270072
% Prof. Dr. Maurício Becerra Vargas
% Robótica Industrial 2024.2

%% Cinemática Inversa
%% Ⅰ) Definição dos Parâmetros
% Comprimentos
l      = [166, 135, 250, 160, 72, 105.64]; %[mm]

% Entrada dos ângulos das juntas em graus pelo usuário:
disp("Coloque os ângulos das juntas em graus:")
th1 = deg2rad(input("θ₁: "));
th2 = deg2rad(input("θ₂: "));
th3 = deg2rad(input("θ₃: "));
th4 = deg2rad(input("θ₄: "));
th5 = deg2rad(input("θ₅: "));

% Resto dos parâmetros
alphas = [0, -pi/2, 0, 0, pi/2, 0];       % α (i-1) [rad]
as     = [0, 0, l(3), l(4), 0, 0];        % a (i-1) [mm]
thetas = [th1, th2-(pi/2), th3, th4+(pi/2), th5, 0]; % θ (i) [rad]
ds     = [l(1)+l(2), 0, 0, 0, l(5), l(6)]; % d (i) [mm]
offsets = [0, -pi/2, 0, +pi/2, 0, 0];
for ii=1:1:6
    L(ii) = RevoluteMDH('alpha',alphas(ii), 'a', as(ii), 'd', ds(ii), 'offset', offsets(ii));
end
myrobot = SerialLink(L,'name','myrobot');
myrobot.teach('rpy/zyx')

Q0 = [0, 0, 0, 0, 0, 0];
Qi = myrobot.ikunc(Ttot,Q0);
Qi = Qi*180/pi;
Js = [Qi(1), Qi(2), Qi(3), Qi(4), Qi(5)+Qi(6)];

fprintf("\n\nCinemática Inversa do Robô: ");
disp(Js)