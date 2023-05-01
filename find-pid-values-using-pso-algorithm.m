close all
clear all
clc


%this code for the estimation of the transfer function of my system
% % Load input-output data
% load('file adress'); % replace with the name of your data file
% 
% % Create a time-series data object
% Ts = 0.01; % sampling time
% data = iddata(y,u,Ts); % y is the output data, u is the input data
% 
% % Specify a model structure
% sys = tfest(data, 3, 1); % 3 poles and 1 zero transfer function


%generate the transfer function 
x=[1];
y=[1 2 3];
g=tf(x,y);
gf=feedback(g,1);
step(gf);
hold on;


%pso constant parameters
c=2;w=0.7;particles=100;iteration=200;var=3; 

%search space
a=0; %lower bound
b=200; %upper bound

%optimization steps
c_cf=0;

%inialization
for m=1:particles    %counter for particles
    for n=1:var      %counter of iteration
        v(m,n)=0;    %initialization of velocity particles
        x(m,n)=a+rand*(b-a);     %initialization of position particles  
        xp(m,n)=x(m,n);
    end

%Model parameters
kp=x(m,1);
ki=x(m,2);
kd=x(m,3);

%simulation Model
gc=pid(kp,ki,kd);
gcf=feedback(gc*g,1);
y=step(gcf);

%ITAE (objective function)
ff1=0;
sim1=size(y);
 for m1=1:sim1
    ff1=ff1+((abs(y(m1)-1))*m1);   %erreur=(y(m1)-1)  % erreur=desired_output-actual_output then the desired_output =1 
 end
  ITAE(m)=ff1; 
end
 
%find the best value
[best_performance,location]=min(ITAE);
fg=best_performance
xg(1)=x(location,1);
xg(2)=x(location,2);
xg(3)=x(location,3);

for i=1:iteration      %counter for iteration
    for m=1:particles  %counter for particles
        for n=1:var    %counter for variables
            v(m,n)=(w*v(m,n))+(c*rand*(xp(m,n)-x(m,n)))+(c*rand*(xg(n)-x(m,n)));    %update velocity
            x(m,n)=x(m,n)+v(m,n);     %update position
        end
        
%check bound

for n=1:var
    if x(m,n)<a
        x(m,n)=a;
    end
    if x(m,n)>b
        x(m,n)=b;
    end
end

%model parameters
kp=x(m,1);
ki=x(m,2);
kd=x(m,3);

%simulation model 
 gc=pid(kp,pi,kd);
 gcf=feedback(g*gc,1);
 y=step(gcf);
 
 %ITAE (objective function)
 ff1=0;
 sim1=size(y);
 for m1=1:sim1
    ff1=ff1+((abs(y(m1)-1))*m1);   %erreur=(y(m1)-1)  % erreur=desired_output-actual_output then the desired_output =1 
 end
  ITAEp(m)=ff1; 
  
  %compare local
   if ITAEp(m)<ITAE(m)
       ITAE(m)=ITAEp(m);
       xp(m,1)=x(m,1);
       xp(m,2)=x(m,2);
       xp(m,3)=x(m,3);
   end
 end

[b_fg,location]=min(ITAE);

%compare global
if b_fg<fg
    fg=b_fg;               %new global value 
    xg(1)=xp(location,1);  %position of var1 
    xg(2)=xp(location,2);  %position of var2
    xg(3)=xp(location,3);  %position of var3
end

    c_cf=c_cf+1;
    best_cf_ac(c_cf)=fg;
end

min_ITAE=fg
kp=xg(1)
ki=xg(2)
kd=xg(3)
    
gc=pid(kp,ki,kd);
gcf=feedback(gc*g,1);
step(gcf); 
    
t_cf=1:c_cf;
figure
plot(t_cf,best_cf_ac,'r--','LineWidth',2);
xlabel('ireration');
ylabel('cost function (ITAE)');
legend('ITAE for pso-pid');
title('ITAE with each iteration');
       

    

