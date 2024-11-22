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

% Entrada das posições em mm pelo usuário:
disp("Coloque as posições em mm do TCP:")
PX = input("X: ");
PY = input("Y: ");
PZ = input("Z: ");
disp("Coloque os ângulos de Euler do robô:");
thz = deg2rad(input("thZ: "));
thy = deg2rad(input("thY: "));
thx = deg2rad(input("thX: "));

% Resto dos parâmetros
alphas = [0, -pi/2, 0, 0, pi/2, 0];       % α (i-1) [rad]
as     = [0, 0, l(3), l(4), 0, 0];        % a (i-1) [mm]
%thetas = [th1, th2-(pi/2), th3, th4+(pi/2), th5, 0]; % θ (i) [rad]
ds     = [l(1)+l(2), 0, 0, 0, l(5), l(6)]; % d (i) [mm]
offsets = [0, -pi/2, 0, +pi/2, 0, 0];

%% Ⅱ) Calculo da Matriz Inversa
Tinv = [cos(thy)*cos(thz), cos(thz)*sin(thx)*sin(thy)-cos(thx)*sin(thz), ...
        cos(thx)*cos(thz)*sin(thy)+sin(thx)*sin(thz), PX; ...
        cos(thy)*sin(thz), cos(thx)*cos(thz)+sin(thx)*sin(thy)*sin(thz), ...
        -cos(thz)*sin(thx)+cos(thx)*sin(thy)*sin(thz), PY; ...
        -sin(thy), cos(thy)*sin(thx), cos(thx)*cos(thy), PZ; ...
        0, 0, 0, 1];


for ii=1:1:6
    L(ii) = RevoluteMDH('alpha',alphas(ii), 'a', as(ii), 'd', ds(ii), 'offset', offsets(ii));
end
myrobot = SerialLink(L,'name','myrobot');
myrobot.teach('rpy/zyx')

% T = [ 1 0 0 PX;
%       0 1 0 PY;
%       0 0 1 PZ;
%       0 0 0 1]

% Ângulos iniciais de referência
Q0 = [0, 0, 85*pi/180, 0, 0, 0]; % Chute inicial

% Calculando os ângulos das juntas
try
    Qi = myrobot.ikunc(Tinv, Q0); % Utilizando a cinemática inversa universal
catch
    error("Não foi possível calcular a cinemática inversa para a posição fornecida.");
end

Qi_deg = rad2deg(Qi);

%% Ⅲ) Resultado
disp("Ângulos das juntas (em graus):");
fprintf("θ₁: %.2f°\n", Qi_deg(1));
fprintf("θ₂: %.2f°\n", Qi_deg(2));
fprintf("θ₃: %.2f°\n", Qi_deg(3));
fprintf("θ₄: %.2f°\n", Qi_deg(4));
fprintf("θ₅: %.2f°\n", Qi_deg(5)+Qi_deg(6));