clear
if not(isfolder('./output/figs'))
    mkdir('./output/figs')
end

%% Constant Parameters.
% Aircraft Constants.
Iy = 27000000.0; W = 500000.0; chat = 27.3; S = 6000.0; 
% Physical Constants.
g = 32.2; rho = 0.0012; m = W./g;
% Aerodynamic Constants.
CD0 = 0.036667; CL0 = 0.5455; CM0 = 0.039;
CD_alpha = 0.26; CL_alpha = 5.2; CM_alpha = -0.74;
CL_deltae = 0.36; CM_deltae = -1.4;
CL_alphar = 2.0; CM_alphar = -8.0;
CL_q = 5.5; CM_q = -22.0;
% Feedback Gains.
K_theta = 0.25; K_u = 40.0;
% Initial Conditions.
v0 = 500.1375; h0 = 20000.0; x0 = 0.0; q0 = 0.0; alpha0 = -0.000055; theta0 = -0.000055; gamma0 = 0.0; deltae0 = 0.027886; T0 = 33005.5;
% State Variables.
v = v0.*cos(alpha0); gamma = gamma0; q = q0; theta = theta0;
% Control Variables.
T_trim = T0; deltae_trim = deltae0; v_trim = v0; theta_trim = theta0;
% Control Coefficients.
deltae = deltae_trim + K_theta.*(theta - theta_trim);
T = T_trim + K_u.*(v - v_trim);
% Angles.
alpha = alpha0; alphar = theta - gamma;

%% Reference Condition.
% Aerodynamic Coefficients.
CL = CL0 + CL_alpha.*alpha + CL_deltae.*deltae + ((chat./2)./v).*(CL_alphar.*alphar + CL_q.*q);
CD = CD0 + CD_alpha.*alpha;
CM = CM0 + CM_alpha.*alpha + CM_deltae.*deltae + ((chat./2)./v).*(CM_alphar.*alphar + CM_q.*q);
% Aerodynamic Forces.
D = 0.5.*rho.*v.^(2).*S.*CD;
L = 0.5.*rho.*v.^(2).*S.*CL;
M = 0.5.*rho.*v.^(2).*S.*(chat./2).*CM;

%% State-Space Form.
% State-Space Model (Elevator and Thrust Input, Pitch Angle Output).
A = [-0.0059 -32.2 0 0; 64.4 0 4353.7 2610.4; 1.1477*10^(-8) 0 -0.2733 -0.1593; 0 0 1 0];
B = [0 6.44*10^(-5); 10441 -1.7715*10^(-6); -0.6734 0; 0 0];
C = [0 0 0 1];
sys_ss = ss(A,B,C,0);

% Plot step response for output due to both inputs on the same graph.
step(deltae*sys_ss(1),T*sys_ss(2)), ylabel('Pitch Angle (rad)')
legend({'$\delta\_{e}$','$\delta\_{T}$'},'Interpreter','latex','Location','northeast')
fig = gcf; fig.PaperPositionMode = 'auto'; fig_pos = fig.PaperPosition; fig.PaperSize = [fig_pos(3) fig_pos(4)];
print('output/figs/ss_step.pdf','-dpdf');
% Plot step response for output due to elevator input.
step(deltae*sys_ss(1)), ylabel('Pitch Angle (rad)')
fig = gcf; fig.PaperPositionMode = 'auto'; fig_pos = fig.PaperPosition; fig.PaperSize = [fig_pos(3) fig_pos(4)];
print('output/figs/ss_step_deltaetrim.pdf','-dpdf');
% Plot step response for output due to thrust input.
step(T*sys_ss(2)), ylabel('Pitch Angle (rad)')
fig = gcf; fig.PaperPositionMode = 'auto'; fig_pos = fig.PaperPosition; fig.PaperSize = [fig_pos(3) fig_pos(4)];
print('output/figs/ss_step_Ttrim.pdf','-dpdf');

%% Transfer-Function Form.
% Transfer-Function Model (coerced from state-space model).
sys_tf = tf(sys_ss);

% Plot bode diagram for output due to elevator input.
margin(sys_tf(1));
fig = gcf; fig.PaperPositionMode = 'auto'; fig_pos = fig.PaperPosition; fig.PaperSize = [fig_pos(3) fig_pos(4)];
print('output/figs/tf_bode_deltaetrim.pdf','-dpdf');
% Plot blode diagram for output due to thrust input.
margin(sys_tf(2));
fig = gcf; fig.PaperPositionMode = 'auto'; fig_pos = fig.PaperPosition; fig.PaperSize = [fig_pos(3) fig_pos(4)];
print('output/figs/tf_bode_Ttrim.pdf','-dpdf');

% Plot root locus for output due to elevator input.
rlocus(sys_tf(1));
fig = gcf; fig.PaperPositionMode = 'auto'; fig_pos = fig.PaperPosition; fig.PaperSize = [fig_pos(3) fig_pos(4)];
print('output/figs/tf_rlocus_deltaetrim.pdf','-dpdf');
% Plot root locus for output due to thrust input.
rlocus(sys_tf(2));
fig = gcf; fig.PaperPositionMode = 'auto'; fig_pos = fig.PaperPosition; fig.PaperSize = [fig_pos(3) fig_pos(4)];
print('output/figs/tf_rlocus_Ttrim.pdf','-dpdf');

P = pole(sys_tf);
Z = zero(sys_tf);
sys_zpk = zpk(sys_tf);

%% Aerodynamic Forces.
% Output due to Elevator Input.
[~,t,x] = step(deltae*sys_ss(1),5);
% Output State Variables.
alpha = x(:,4) - x(:,2); deltae = deltae_trim + K_theta.*(x(:,4) - theta_trim); q = x(:,3); v = x(:,1).*cos(x(:,1));
% Aerodynamic Coefficients.
CL = CL0 + CL_alpha.*alpha + CL_deltae.*deltae + ((chat./2)./v).*(CL_alphar.*alphar + CL_q.*q);
CD = CD0 + CD_alpha.*alpha;
CM = CM0 + CM_alpha.*alpha + CM_deltae.*deltae + ((chat./2)./v).*(CM_alphar.*alphar + CM_q.*q);
% Aerodynamic Forces.
D = 0.5.*rho.*v.^(2).*S.*CD;
L = 0.5.*rho.*v.^(2).*S.*CL;
M = 0.5.*rho.*v.^(2).*S.*(chat./2).*CM;
% Plot aerodynamic forces against time on the same plot for elevator input.
plot(t,D,t,L,t,M)
xlabel('Time (seconds)'), ylabel('Force (slugs)'), xlim([0 max(t)]), xticks(0:1:max(t))
title('Aerodynamic Forces'), legend({'$D$','$L$', '$M$'},'Interpreter','latex','Location','northeast')
fig = gcf; fig.PaperPositionMode = 'auto'; fig_pos = fig.PaperPosition; fig.PaperSize = [fig_pos(3) fig_pos(4)];
print('output/figs/aeroforces_deltaetrim.pdf','-dpdf');

% Output due to Thrust Input.
[~,t,x] = step(T*sys_ss(2),5);
% Output State Variables.
alpha = x(:,4) - x(:,2); deltae = deltae_trim + K_theta.*(x(:,4) - theta_trim); q = x(:,3); v = x(:,1).*cos(x(:,1));
% Aerodynamic Coefficients.
CL = CL0 + CL_alpha.*alpha + CL_deltae.*deltae + ((chat./2)./v).*(CL_alphar.*alphar + CL_q.*q);
CD = CD0 + CD_alpha.*alpha;
CM = CM0 + CM_alpha.*alpha + CM_deltae.*deltae + ((chat./2)./v).*(CM_alphar.*alphar + CM_q.*q);
% Aerodynamic Forces.
D = 0.5.*rho.*v.^(2).*S.*CD;
L = 0.5.*rho.*v.^(2).*S.*CL;
M = 0.5.*rho.*v.^(2).*S.*(chat./2).*CM;
% Plot aerodynamic forces against time on the same plot for thrust input.
plot(t,D,t,L,t,M)
xlabel('Time (seconds)'), ylabel('Force (slugs)'), xlim([0 max(t)]), xticks(0:1:max(t))
title('Aerodynamic Forces'), legend({'$D$','$L$', '$M$'},'Interpreter','latex','Location','northeast')
fig = gcf; fig.PaperPositionMode = 'auto'; fig_pos = fig.PaperPosition; fig.PaperSize = [fig_pos(3) fig_pos(4)];
print('output/figs/aeroforces_Ttrim.pdf','-dpdf');
