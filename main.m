% Parámetros del controlador
Kp = 1; % Ganancia proporcional
Ki = 0.1; % Ganancia integral
Kd = 0.05; % Ganancia derivativa

% Condiciones iniciales
x0 = [0; 0]; % Posición y velocidad inicial
r = 10; % Valor de referencia

% Función de simulación del sistema
sys = @(t, x, u) [x(2); -x(1) + u];

% Tiempo de simulación
tspan = [0 20];

% Función de control PID
function u = pid_control(t, x, r, Kp, Ki, Kd, e, e_ant)
  % Cálculo del error y la derivada del error
  e = r - x(1);
  de = (e - e_ant) / (t - t_ant);
  
  % Cálculo de la acción de control
  u = Kp * e + Ki * trapz(e) + Kd * de;
  
  % Actualización de las variables auxiliares
  t_ant = t;
  e_ant = e;
end

% Simulación del sistema con control PID
[t, x] = ode45(@(t, x) sys(t, x, pid_control(t, x, r, Kp, Ki, Kd, 0, 0)), tspan, x0);
