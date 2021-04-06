function AttitudePropagate( J,q_0,w_0,h_0,N_0,n )

%definition of state and control vectors
% w  = [w1_0; w2_0; w3_0];
% q  = [q1_0; q2_0; q3_0; q0_0];
% h  = [h1_0; h2_0; h3_0];
% N  = [N_d; N_c; N_w]
% n  = numvber of steps, n=1 corresponds to initial conditions

% Build time step matrix of states and controls
w(:,1)=w_0;
q(:,1)=q_0;
h(:,1)=h_0;
N(:,1)=N_0;
%given J, q_0, w_0, h_0, N_0

for t=2:1:n
    % Step
    [ q(:,t) w(:,t) h(:,t) ] = AttitudePropagateStep( J, q(:,(t-1)), w(:,(t-1)), h(:,(t-1)), N(:,(t-1)) );
    
    % Control N vector
    N(:,t)=Nupdate(J,q(:,t),w(:,t),h(:,t),N(:,t-1));

end

end

function [ q_1 w_1 h_1 ] = AttitudePropagateStep( J,q,w,h,N )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

% Initialize


% Omega_KEOM = [    0  w3_0 -w2_0 w1_0
%               -w3_0     0  w1_0 w2_0
%                w2_0 -w1_0     0 w3_0
%               -w1_0 -w2_0 -w3_0    0];
%           
% Omega_DEOM = [    0 -w3_0 w2_0
%                w3_0     0 w1_0
%               -w2_0  w1_0    0];
          
% Function Handles

DEOM = @(w_1,h_1,N_1) J^-1*N(1) + J^-1*N(2) + J^-1*[0 -w(3) w(2); w(3) 0 w(1); -w(2) w(1) 0]*J*w - cross(w,h) + J^-1*N(3);
KEOM = @(w_q,q_11) 0.5.*[0, w(3), -w(2), w(1); -w(3), 0,  w(1), w(2); w(2), -w(1), 0, w(3); -w(1), -w(2), -w(3), 0]*q;




% Rigid Body Dynamic Equation of Motion - Runge Kutta

[tw1, w1]=ode45(DEOM, [0,1] ,w);

[w1,q1]=ode45(KEOM, [w,w1], q);


end

function [N_1] = Nupdate(J,q,w,h,N)

% controller goes here

N_1=N; %placeholder

end