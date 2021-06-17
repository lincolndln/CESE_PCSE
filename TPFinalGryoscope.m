% Trabajo Final Protocolos de comunicacion en sistemas embebidos
% 
clc     % Se limpia ventana de comandos
% Se busca y elemina una interface de comunicacion en puerto COM4
delete(instrfind({'port'}, {'COM4'}));

% Se construye objeto serial en COM4 y se indica velocidad de transmisión
miPuerto = serial('COM4', 'BaudRate', 115200);      

fopen(miPuerto);        % Conecta el objeto serial al puerto 


% Se dibuja una grafica para
x = -2:0.2:2;               % Limites en eje x
y = -2:0.2:2;               % Limites en eje y
[X, Y] = meshgrid(x,y);     % Se genera una matriz
Z = X.*exp(-X.^2 - Y.^2);   % Se obtiene los valores Z de la matriz
surfc(X, Y, Z);             % Se grafica una superficie

xlabel("eje X")         % Etiqueta en eje X
ylabel("eje Y")         % Etiqueta en eje Y
zlabel("eje Z")         % Etiqueta en eje Z

azimuth = -37.5;        % Valor por defecto, rotacion sobre eje z
elevation = 30;         % Valor por defecto, rotacion sobre eje x

% Se indica superficie con orientacion seleccionada
view([azimuth elevation])

contador = 100;         % Variable contador 

% Bucle infinito
while true
    entradPuerto = fscanf(miPuerto, '%f');    % Se capturan datos
    if (contador == 1)  %--- X - Elevation    % Corresponde a giro sobre X
        elevation = elevation + entradPuerto; % Se añade a elevación
    end
    if (contador == 2)  %--- Y
        % NO usado
    end
    if (contador == 3)  %--- Z - Azimuth      % Corresponde giro sobre X
         azimuth = azimuth - entradPuerto;    % Se añade a azimuth
    end
    view([azimuth elevation])       % Se indica orientacion
    drawnow                         % Se dibuja la nueva posicion d

    contador = contador + 1;
    
    if (entradPuerto > 10000)       % 10000 = Indicando nuevos datos
        contador = 1;
    end
end
% =====================[ Fin del archivo ]=====================

