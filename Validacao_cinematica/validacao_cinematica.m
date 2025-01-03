clc; clear; close all;
% Carlos Augusto Fernandes Leitão        - 211270628
% Cesar Augusto Mendes Cordeiro da Silva - 211270121
% Guilherme Bueno Guidetti               - 211270601
% Lucas de Camargo Mainente              - 211270661
% Rafael Kenji Issaka                    - 201270072
% Prof. Dr. Maurício Becerra Vargas
% Robótica Industrial 2024.2

%% Validação Cinemática
%% Ⅰ) Cinemática Direta:
run("Cinematica_direta.m");

%% Ⅱ) Cinemática Inversa:
offsets = [0, -pi/2, 0, +pi/2, 0, 0];

for ii=1:1:6
    L(ii) = RevoluteMDH('alpha',alphas(ii), 'a', as(ii), 'd', ds(ii), 'offset', offsets(ii));
end
myrobot = SerialLink(L,'name','myrobot');
myrobot.teach('rpy/zyx')

% Ⅲ) Ângulos iniciais de referência
Q0 = [0, 0, 0, 0, 0, 0]; % Chute inicial

% Ⅳ) Calculando os ângulos das juntas
try
    Qi = myrobot.ikunc(Ttotal, Q0); % Utilizando a cinemática inversa universal
catch
    error("Não foi possível calcular a cinemática inversa para a posição fornecida.");
end

Qi_deg = rad2deg(Qi);

disp("---------------------------------------------------------------");
disp("Ângulos das juntas (em graus):");
fprintf("θ₁: %.2f°\n", Qi_deg(1));
fprintf("θ₂: %.2f°\n", Qi_deg(2));
fprintf("θ₃: %.2f°\n", Qi_deg(3));
fprintf("θ₄: %.2f°\n", Qi_deg(4));
fprintf("θ₅: %.2f°\n", Qi_deg(5)+Qi_deg(6));