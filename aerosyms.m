clear

%% Symbolic Variable Definitions.
% State Variables.
syms v gamma q theta
% Control Variables.
syms T_trim deltae_trim v_trim theta_trim
% Aircraft Constants.
syms Iy W chat S alpha alphar
% Physical Constants.
syms g rho m
% Aerodynamic Constants.
syms CD0 CL0 CM0 
syms CD_alpha CL_alpha CM_alpha
syms CL_deltae CM_deltae
syms CL_alphar CM_alphar
syms CL_q CM_q
% Feedback Gains.
syms K_theta K_u
% Functions.
syms deltae T D L M
% Differential Equations.
syms f1 f2 f3 f4

%% Symbolic Function Definitions.
% Control Functions.
deltae = deltae_trim + K_theta*(theta - theta_trim);
T = T_trim + K_u*(v - v_trim);
% Aerodynamic Functions.
D = 0.5*rho*v^2*S*(CD0 + CD_alpha*alpha);
L = 0.5*rho*v^2*S*(CL0 + CL_alpha*alpha + CL_deltae*deltae + ((chat/2)/v)*(CL_alphar*alphar + CL_q*q));
M = 0.5*rho*v^2*S*(chat/2)*(CM0 + CM_alpha*alpha + CM_deltae*deltae + ((chat/2)/v)*(CM_alphar*alphar + CM_q*q));

%% Symbolic Differential Equation Definitions.
f1 = (T*cos(alpha) - D - W*sin(gamma))*(1/m);
f2 = (T*sin(alpha) + L - W*cos(gamma))*(1/m*v);
f3 = M*(1/Iy);
f4 = q;

%% Differentiation.
syms df1dv df2dv df3dv df4dv
syms df1dgamma df2dgamma df3dgamma df4dgamma
syms df1dq df2dq df3dq df4dq
syms df1dtheta df2dtheta df3dtheta df4dtheta
syms df1ddeltae df2ddeltae df3ddeltae df4ddeltae
syms df1dT df2dT df3dT df3dT

df1dv = diff(f1,v); df2dv = diff(f2,v); df3dv = diff(f3,v); df4dv = diff(f4,v);
df1dgamma = diff(f1,gamma); df2dgamma = diff(f2,gamma); df3dgamma = diff(f3,gamma); df4dgamma = diff(f4,gamma);
df1dq = diff(f1,q); df2dq = diff(f2,q); df3dq = diff(f3,q); df4dq = diff(f4,q);
df1dtheta = diff(f1,theta); df2dtheta = diff(f2,theta); df3dtheta = diff(f3,theta); df4dtheta = diff(f4,theta);
df1ddeltae = diff(f1,deltae_trim); df2ddeltae = diff(f2,deltae_trim); df3ddeltae = diff(f3,deltae_trim); df4ddeltae = diff(f4,deltae_trim);
df1dT = diff(f1,T_trim); df2dT = diff(f2,T_trim); df3dT = diff(f3,T_trim); df4dT = diff(f4,T_trim);
