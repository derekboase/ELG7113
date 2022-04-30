clear
clc
close all

g = 9.81;
Rho = 1.2;
m = 4/7000;
r = 38/2000;
veq = 2.8;

Vb = (4/3)*pi*r^3;
b = (2*g/veq)*((m - Rho*Vb)/m);

Omega = 1;
Zeta = 1;

Ts=0.2;

a1= -1.26797;
a2= 0.267972;
b0= 0.0888233;
b1= 0.0575823;

am1= -1.63746;
am2= 0.67032;
bm0= 0.0175231;
bm1= 0.0153354;


% Perfect Model Following Parameters
theta10 = -Omega^2/b;
theta20 = (2*Zeta*Omega - b)/b;

Gamma=400;

% Initializations
ym(1)=0; % for k=0
ym(2)=0; % for k=1

y(1)=0; % for k=0
y(2)=0; % for k=1

theta1(1)=0; % for k=0
theta2(1)=0; % for k=0

theta1(2)= theta1(1)-Gamma*(y(1)-uc(0))*(y(2)-y(1)-ym(2)+ym(1)); % for k=1
theta2(2)= theta2(1)+Gamma*((y(2)-y(1))/Ts)*(y(2)-y(1)-ym(2)+ym(1)); % for k=1

% Recursive Algorithm
maxt=90;

t=0:Ts:maxt;

k=0;
ki=k+1;

while(k<length(t)-2)
    ym(ki+2)= -am1*ym(ki+1)-am2*ym(ki)+bm0*uc((k+1)*Ts)+bm1*uc(k*Ts);
    y(ki+2)= (1/(1+b0*theta2(ki+1)/Ts))*(-a1*y(ki+1)-a2*y(ki)+b0*(theta1(ki+1)*(y(ki+1)-uc((k+1)*Ts))+theta2(ki+1)*y(ki+1)/Ts)+b1*(theta1(ki)*(y(ki)-uc(k*Ts))-theta2(ki)*((y(ki+1)-y(ki))/Ts))) + normrnd(0,0.001);
     
    theta1(ki+2)= theta1(ki+1)-Gamma*(y(ki+1)-uc((k+1)*Ts))*(y(ki+2)-y(ki+1)-ym(ki+2)+ym(ki+1));
    theta2(ki+2)= theta2(ki+1)+Gamma*((y(ki+2)-y(ki+1))/Ts)*(y(ki+2)-y(ki+1)-ym(ki+2)+ym(ki+1));
    
    k=k+1;
    ki=ki+1;
end

for i=1:length(t)-1
    v(i)=theta1(i)*(y(i)-uc((i-1)*Ts))-theta2(i)*((y(i+1)-y(i))/Ts);
end  

for j=0:(length(t)-1)
    Uc(j+1)=uc(j*Ts);
end

% Plots
figure
plot(t,ym)
xlabel('t');
ylabel('ym');
grid on;

figure
plot(t,ym,t,y)
xlabel('t');
legend('ym','y')
grid on;

figure
plot(t,Uc,t,y)
xlabel('t');
legend('uc','y')
grid on;

figure
plot(t,y-ym)
xlabel('t');
ylabel('y-ym');
grid on;

figure
plot(t,theta10*ones(1,length(t)),t,theta1);
xlabel('t');
ylabel('theta1');
grid on;

figure
plot(t,theta20*ones(1,length(t)),t,theta2);
xlabel('t');
ylabel('theta2');
grid on;

figure
stairs(t(1:length(t)-1),v);
xlabel('t');
ylabel('v');
grid on;

figure
stairs(t(1:length(t)-1),v+veq*ones(1,length(t)-1))
xlabel('t');
ylabel('vf');
grid on;

figure
subplot(1,2,1)
plot(t,theta10*ones(1,length(t)),t,theta1);
xlabel('t');
ylabel('theta1');
grid on;

subplot(1,2,2)
plot(t,theta20*ones(1,length(t)),t,theta2);
xlabel('t');
ylabel('theta2');
grid on;

figure
subplot(1,2,1)
stairs(t(1:length(t)-1),v);
xlabel('t');
ylabel('v');
grid on;

subplot(1,2,2)
stairs(t(1:length(t)-1),v+veq*ones(1,length(t)-1))
xlabel('t');
ylabel('vf');
grid on;