clear all; close all; clc
s=tf('s');

% Capstone Impulse Disturbance
%% Plant

%Parameters
B=0.001765/1000/0.104719755; %Motor Damping [N*m*s/rad]
Rp=0.02184; %Radius of Pulley [m]
Jm=0.05649/1000; 
Jl=0.1*(Rp^2); Jt=Jm+Jl; %Motor Inertia [N*m/s/s]
theta0=pi(); %rad. max step.
imax=10; %[A]

% Transfer function
Ka=0.4; %Current Amplifier [A/V]
Km=0.09674; %Motor Constant [N*m/A]
Integrate=1/s; %[Theta/ThetaDot]

Kda=Km*Ka*Rp/B;
taum=Jt/B;
Tm=Kda/(s*(taum*s+1)); %Plant Transfer Function [m/V]

%% Controller
%design goals:
%under 10%OS, so zeta>.59, so PM>59deg
Pm_desired=59;

%finding margin of Tm alone
[Gm,Pm,Wgm,Wpm]=margin(Tm);%finding margins of Tm alone
[mag,phase,wout]=bode(Tm);%uncompensated values

Gain_pdesign=interp1(squeeze(phase),squeeze(mag),Pm_desired-180);
Kp=1/Gain_pdesign;
Pm_needed=59-Pm+5;
beta=(1-sin(Pm_needed*pi()/180))/(1+sin(Pm_needed*pi()/180));
K=imax*beta/Ka/theta0;
mag_gc_jw_max=K/sqrt(beta);
mag_gc_dB=20*log10(K)-10*log10(beta);
w_max=interp1(20*log10(squeeze(mag)),squeeze(wout),-mag_gc_dB);
T=1/(w_max*sqrt(beta));

%compensator equation
k_c=K*(1/beta); %K is limited by max current
z_c=1/T;
p_c=1/(beta*T);
Controller=k_c*(s+z_c)/(s+p_c);

%% Test
% System=feedback(Controller*Tm,1);
% figure
% plot(step(System))

%% C Biquad Header File & Butterworth Filter
PrintToFile = 1; % Set to 0 to print header file in Command Window only

HeaderFileName = ...
'\Users\Mark Chang\Documents\Mechatronics\CAPSTONE\Impulse_d\Impulse.h';
fs=4000;        %---Sampling frequency for discrete filter
Sampling_Period=1/fs; 
Controller_d=c2d(Controller,Sampling_Period,'tustin'); %Discretizing

[b,a]=tfdata(Controller_d,'v');       %---get discrete system coefficients
[sos,gain]=tf2sos(b,a);     %---convert to biquads
[ns,n]=size(sos);

for j=1:3                   %---Apply the gain to the final biquad
    sos(ns,j)=gain*sos(ns,j);
end

if PrintToFile
    fid=fopen(HeaderFileName,'W');    
else
    fid=1;   
end

% Butterworth Filter
cutoff=30; %[Hz]
cutoff=cutoff/(fs/2); %[pi*rad/sample]
nthorder=2; %high order may eat up margin
[b,a]=butter(nthorder, cutoff);
[butter_sos,gain]=tf2sos(b,a);     %---convert to biquads
[ns,n]=size(butter_sos);

for j=1:3                   %---Apply the gain to the final biquad
    butter_sos(ns,j)=gain*butter_sos(ns,j);
end

if PrintToFile
    fid=fopen(HeaderFileName,'W');    
else
    fid=1;   
end



%%
%---Structure for cascade
comment='Impulse Disturbance Lead-Lag Compensator Biquad';
PrintTo2BiquadFHeaderFile(fid, sos, 'ImpulseController', butter_sos, 'Butter', Sampling_Period, comment);

if fid~=1
    fclose(fid);
end 
return
1+1
%% Trajectory Generation
%NOTE: deltaT should change depending on the desired speed

Starting_Position=0; %[m]
Desired_Position=0.3; %[m]
deltaT=2; %Time it takes to move from start to desired[s]

a0=Starting_Position;
a1=0;
a2=3*(Desired_Position-Starting_Position)/(deltaT^2);
a3=-2*(Desired_Position-Starting_Position)/(deltaT^3);

t=linspace(0,2,100);
Trajectory=a0+a1*t+a2*(t.^2)+a3*(t.^3);
% figure
% plot(t,Trajectory)

printf('done')

