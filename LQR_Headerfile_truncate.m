%%
clc; clear; close all;

%Initialize parameters

%Motor Specs
Kt = 0.0967; %Nm/A
Kvi = 0.4; %A/V
K_iden = 0.4553;
Kvi = Kvi*K_iden;
rp = 0.021844; %pulley radius

m1 = 0.382;
motor_I = 0.05649/1000; %kg*m^2
m2 = 1.3215 +motor_I/rp^2;
L = 0.7;
g= 9.81;
b = 0.1;
theta_i =0;
X_Ball = 0;
Rm = 1.6;

tau = 0.;
% tau = 0;

% J_fw = 0.000109
% J_m = 2.7100e-04
%Setting up statespace matrices [x, v, theta, w]. Noted that the
%state variables are at different order (similar to the UMich's order)
A = [0 1 0 0;
    0 0 -g 0;
    0 0 0 1
    0 b/(L*m1) -g*(m1+m2)/(L*m1) 0];

%[omega; theta; velocity; x_trolley]
% A_l = [0 -g*(m1+m2)/(L*m1) b/(L*m1) 0;
%     1 0 0 0;
%     0 -g 0 0;
%     0 0 1 0];
%State Space Matrix B
% B_l = [-1/(L*m1) 1/(L*m2); 0 0; 0 1/m2; 0 0];

B = [0 ; 0; 0; -1/(L*m1)];

C = [1 0 0 0;
    0 0 1 0];

D = [0;
    0];

%Defining the statespace system to MATLAB
states = {'x' 'x_dot' 'phi' 'phi_dot'};
inputs = {'u'};
outputs = {'x_ball'; 'theta' };

sys_ss = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);


poles = eig(A);
co = ctrb(sys_ss);
%A quick way to test whether the system is well-defined or not is calculate
%the rank. If the rank equals the matrix dimension, then the state matrix
%is non-singular = good
controllability = rank(co); 

%% LQR controls

%LQR controls takes feedbacks from every state variables and applies a
%proportional gain to each of these. It tries to drive all state variables
%to a reference value called r. Usually, this controls aims to drive every
%state variable to zero because driving them to a non-zero value requires
%additional effort.

%The designer tunes the controller by changing paramerter in the Q and R
%matrix

%Declare the Q matrix - the designer would tune this matrix primarily, all
%of the diagonal elements corresponds to how much the designer want to
%"punish" the error of each state variable in the same order
%[x,v,theta,omega]. For example, if the designer want to pushish the error
%in the ball position, he can increase the value of Q(1,1)
Q = zeros(4,4);
Q(1,1) = 3000;
Q(2,2) = 0;
Q(3,3) = 200;
Q(4,4) = 0;
Q(1,3) = 0;
Q(3,1) = 0;
%Generally, increasing the diagonal values leads to better transient
%response but demand more input power.

%Declaring the R matrix - The R matrix represents how much the designer 
%wants to amplify the input directly. Decrease the input value -> 
%requesting more input power
R = 0.1;
PID_Sw = 0;
Ctrl_Sw = 1;
%Using MATLAB LQR function to calculate all the gains given the statespace,
%Q, and R matrices.
K = lqr(A,B,Q,R); %Gain matrice
K1 = K(1);
K3 = K(3);
Kr = K;
Kr(3) = 0;
%Rewrite the statespace matrices with the addition of the feedbacks 
Ac = A-B*K;
Bc = B;
Cc = C;
Dc = D;

%In reality, the system won't be able to acquire feedbacks from every state
%variables, so we design an estimator to estimate all of the state 
%variables based on the system's output and input. 

%For the estimator to work effectively, it has to converge to the actual
%output value a lot faster than how the output converges to the reference
%value. One way to resolve this is to calculate the poles of the system and
%makes the poles of the estimator 4->10 times larger than the "slowest" 
%pole (smallest absolute value).

poles = eig(Ac); %Calculating poles of the system

% P = real(min(poles))*[7 8 9 10]; %Calculating the poles of the estimator
P = [-60 -61 -62 -63];
%The "place" function calculate the feedback gain of the estimator based on
%the state matrices and desired poles' locations.
Lp = place(A',C',P)';

%Redefine the statespace matrices taking into account the estimator and the
%K gain matrix
Ace = [(A-B*K) (B*K);
       zeros(size(A)) (A-Lp*C)];
Bce = [B;
       zeros(size(B))];
Cce = [Cc zeros(size(Cc))];
Dce = D;

states = {'x' 'x_dot' 'phi' 'phi_dot' 'e1' 'e2' 'e3' 'e4'};
%e1 e2 e3 e4 are the estimator errors for each state variables
inputs = {'r'};
outputs = {'xball';'theta'};

%Create the system from the state matrices
sys_est_cl = ss(Ace,Bce,Cce,Dce,'statename',states,'inputname',inputs,'outputname',outputs);

t = 0:0.01:5;
r = zeros(size(t));
%Simulate the system with initial condition of the velocity of the ball of
%0.7 m/s
xo = [0; 0.7; 0; 0; 0; 0.7; 0; 0]; %[x;v;phi,omega;e1;e2;e3;e4]
[y,t,x]=lsim(sys_est_cl,r,t,xo);
% figure(1)
% [AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
% hold on
% set(get(AX(1),'Ylabel'),'String','ball position (m)')
% set(get(AX(2),'Ylabel'),'String','Angle (rad)')
% title('Transient Response with Initial Condition(s)')

%% Converting to TF
A_cv = A+B*(-K)+Lp*(-C);
B_cv = Lp;
C_cv = -K;
D_cv = [0 0];
[n1,d1] = ss2tf(A_cv,B_cv,C_cv,D_cv,1);
[n2,d2] = ss2tf(A_cv,B_cv,C_cv,D_cv,2);


filter = tf([1],[tau 1]);
tf1 = tf(n1,d1);
tf1 = tf1*rp/Kt/Kvi;

tf2 = tf(n2,d2)*tf([1],[tau 1]);
tf2 = tf2*rp/Kt/Kvi;

%% Controller Constants
Kp = 346;
Kp_PI = 276;
Ki = 736;
Ki_PI = 645;
Kd = 40;
Kn = 447;
Ki_FB = 0.5;
% %Motor Specs
% Kt = 0.11;
% Kvi = 0.79; 
% rp = 0.021844; %pulley radius

P_Ic = 0;   %Input position reference
Ic_x = 0;
Ic_theta = asin(Ic_x/L);
Vlim = 10; %Voltage limit
mu = 0.1;
Fs = mu*(m1+m2)*g;

%%
%---
PrintToFile = 1; % Set to 0 to print header file in Command Window only

HeaderFileName1 = ...
'C:\Users\micha.DESKTOP-9M5IU3Q\Dropbox\ME 494 - Capstone\Programming\myLab7\Controller4000x.h';

%---define the 3rd order system
%-----continuous:
fs=4000;        %---Sampling frequency for discrete filter
T=1/fs;         %---Sampling period

%-----discrete equivalent:
% tfz_filter = c2d(filter,T,'tustin');
s = tf('s');
tfz_1=c2d(tf1,T,'tustin');
tfz_2=c2d(tf2,T,'tustin');
tfz_3=c2d(Ki_FB/s,T,'tustin');
%---Bode diagrams
% f=logspace(0,2.9,100);
% w=2*pi*f;
% [mc,pc,wc]=bode(sc,w);
% [md,pd,wd]=bode(sd,w);
% subplot(221)
%     semilogx(wc/2/pi,squeeze(db(mc)),wd/2/pi,squeeze(db(md))); grid
%     title('Bode Diagram');     ylim([-50,10]);
% subplot(223)
%     semilogx(wc/2/pi,squeeze(pc),wd/2/pi,squeeze(pd)); grid
%     xlabel('Frequency - Hz'); 
    
%---continuous and discrete responses:
% [yd,t]=step(sd);
% [yc,t]=step(sc,t);
% npts=length(t);
% subplot(122)
%     plot(t,yc,t,yd); legend('continuous','discrete')
%     grid
%     xlabel('Time - s'); title('unit step response')

%---Biquad Cascade
% SOS is an L by 6 matrix with the following structure:
%         SOS = [ b01 b11 b21  1 a11 a21  
%                 b02 b12 b22  1 a12 a22
%                 ...
%                 b0L b1L b2L  1 a1L a2L ]
[b1,a1]=tfdata(tfz_1,'v');       %---get discrete system coefficients
[sos1,gain1]=tf2sos(b1,a1);     %---convert to biquads
[ns1,n1]=size(sos1);
for j=1:3                   %---Apply the gain to the final biquad
    sos1(ns1,j)=gain1*sos1(ns1,j);
end



[b2,a2]=tfdata(tfz_2,'v');       %---get discrete system coefficients
[sos2,gain2]=tf2sos(b2,a2);     %---convert to biquads
[ns2,n2]=size(sos2);
for j=1:3                   %---Apply the gain to the final biquad
    sos2(ns2,j)=gain2*sos2(ns2,j);
end

[b3,a3]=tfdata(tfz_3,'v');       %---get discrete system coefficients
[sos3,gain3]=tf2sos(b3,a3);     %---convert to biquads
[ns3,n3]=size(sos3);
for j=1:3                   %---Apply the gain to the final biquad
    sos3(ns3,j)=gain3*sos3(ns3,j);
end

if PrintToFile
    fid1=fopen(HeaderFileName1,'W');    
else
    fid1=1;   
end

%---Structure for cascade
comment=['TF for input 1'];
PrintTo3BiquadFHeaderFile(fid1, sos1, 'controller1',sos2, 'controller2',sos3, 'controller3' ,T, comment);

if fid1~=1
    fclose(fid1);
end 

% [b2,a2]=tfdata(tfz_2,'v');       %---get discrete system coefficients
% [sos2,gain2]=tf2sos(b2,a2);     %---convert to biquads
% [ns2,n2]=size(sos2);
% for j=1:3                   %---Apply the gain to the final biquad
%     sos2(ns2,j)=gain2*sos2(ns2,j);
% end
% 
% if PrintToFile
%     fid2=fopen(HeaderFileName2,'W');    
% else
%     fid2=1;   
% end
% 
% %---Structure for cascade
% comment=['TF for input 2'];
% PrintToBiquadFHeaderFile(fid2, sos2, 'myFilter2', T, comment);
% 
% if fid2~=1
%     fclose(fid2);
% end 
% return
