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
l      = [166      , 135 ,250  , 160  , 72   , 105.64]; %[mm]

% Entrada dos ângulos das Juntas em graus pelo usuário:
disp("Coloque os ângulos das juntas em graus:")
th1 = deg2rad(input("θ₁: "));
th2 = deg2rad(input("θ₂: "));
th3 = deg2rad(input("θ₃: "));
th4 = deg2rad(input("θ₄: "));
th5 = deg2rad(input("θ₅: "));

% Resto dos parâmetros
alphas = [0        ,-pi/2     , 0   , 0         , pi/2, 0]; % α (i-1) [rad]
as     = [0        ,0         , l(3), l(4)      , 0  , 0];   % a (i-1) [mm]
thetas = [th1      ,th2-(pi/2), th3 , th4+(pi/2), th5, 0];    % θ (i) [rad]
ds     = [l(1)+l(2),0         , 0   , 0         , l(5), l(6)]; % d (i) [mm]

%% Ⅱ) Calculo da Matriz de Transformação Homogênea

T = zeros(4, 4, 6);

for ii=1:1:6
    T(:,:,ii) = [cos(thetas(ii)), -sin(thetas(ii)), 0, as(ii); ...
                (sin(thetas(ii))*cos(alphas(ii))), (cos(thetas(ii))*cos(alphas(ii))), (-sin(alphas(ii))), (-sin(alphas(ii))*ds(ii)); ...
                (sin(thetas(ii))*sin(alphas(ii))), (cos(thetas(ii))*sin(alphas(ii))), (cos(alphas(ii))), (cos(alphas(ii))*ds(ii)); ...
                0, 0, 0, 1];
end

Ttotal = T(:,:,1)*T(:,:,2)*T(:,:,3)*T(:,:,4)*T(:,:,5)*T(:,:,6);

disp("---------------------------------------------------------------");
disp("Matriz de Transformação Homogênea do Robô RV-2AJ [5DOF]:");
Ttotal = round(Ttotal,2,"decimals")

%% Ⅲ) Cálculo das Coordenadas, Roll (B) e Pitch (A);
X = Ttotal(1, 4);
Y = Ttotal(2, 4);
Z = Ttotal(3, 4);
Roll = rad2deg(th2 + th3 + th4);
Pitch = rad2deg(th5 + th1 * cos(deg2rad(Roll)));

disp("---------------------------------------------------------------");
disp("Posição e Orientação do RV-2AJ [5DOF] - Conforme Teach Pendant:");
fprintf('X         = %.2f mm\n', X);
fprintf('Y         = %.2f mm\n', Y);
fprintf('Z         = %.2f mm\n', Z);
fprintf('Pitch (A) = %.2f ∘\n', Pitch);
fprintf('Roll  (B) = %.2f ∘\n', Roll);

%% Ⅳ) Calculo dos Ângulos de Euler
if Ttotal(3,1) < 1 
    if Ttotal(3,1) > -1
        thY = asin(Ttotal(3,1));
        thZ = atan2(Ttotal(2,1), Ttotal(1,1));
        thX = atan2(Ttotal(3,2), Ttotal(3,3));
    else
        thY = pi/2;
        thZ = -atan2(Ttotal(2,3), Ttotal(2,2));
        thX = 0;
    end
else
    thY = -pi/2;
    thZ = atan2(Ttotal(2,3), Ttotal(2,2));
    thX = 0;
end
thX = rad2deg(thX);
thY = -rad2deg(thY);
thZ = rad2deg(thZ);

disp("---------------------------------------------------------------");
disp("Ângulos de Euler do RV-2AJ [5DOF]:");
fprintf("θX = %.2f ∘\n", thX);
fprintf("θY = %.2f ∘\n", thY);
fprintf("θZ = %.2f ∘\n", thZ);



% offset = [l(1)+l(2), 0    , 0   , 0    , l(3) , l(6)]; %












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