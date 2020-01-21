clc
close all
clear all

load('Kinematics.mat')

q = eps*ones(1,7);
q1 = q(1);
q2 = q(2);
q3 = q(3);
q4 = q(4);
q5 = q(5);
q6 = q(6);
q7 = q(7);
hq = eval(h);
hd = [0.5;0.5;0.5];
hdp = 0;
k1 = [5;5;5];
k2 = [1;1;1];
lamda = 0.1;
pointL = [0.4716;0.315;1.04];
pointL = zeros(3,1);
H(:,1) = hq;
eh(:,1) = [0;0;0];
Q(1,:) = q;

%%
damp =1;
kd = 10;
i = 2;
m = 1;
dt = 0.1;
tau = 5;
for i=2:1:30
    q1 = q(1);
    q2 = q(2);
    q3 = q(3);
    q4 = q(4);
    q5 = q(5);
    q6 = q(6);
    q7 = q(7);
    hq = eval(h);
    H(:,i) = hq; 
    eh(:,i) = hd - hq;
    hv = k1 .* tanh(k2 .* eh(:,i));
    JA1 = eval(JA);
    JA1 = JA1';
    J1 = JA1.' * JA1 + lamda*eye(3);
    J2 = inv(J1);
    J = J2 * JA1.' ;
    s = hdp + hv;
    %Jinv = pinv(JA1)



    %Force
    d = pointL - hq;
    v = H(:,i) - H(:,i-1);
    ur = d/norm(d);
    Fd = -damp*(v'*ur)*ur;
    Fk = -kd*d;
    dobsgoal = hd - pointL;
    dactualgoal = hq - hd;
    plan = cross(dactualgoal, dobsgoal);
    plan = plan/ norm(plan);
    force = cross(plan ,Fk );
    Ft = Fk + force;
    a = Ft/m;
    vf = v + a *dt;


    C = J.' * ((1- exp(-tau*norm(d)))*s +  exp(-tau*norm(d))*vf);
    q = q + 0.1*C.';
    Q(i,:) = q;
end

figure(1)
subplot(1,3,1); plot(eh(1,:));
subplot(1,3,2); plot(eh(2,:));
subplot(1,3,3); plot(eh(3,:));

figure(2)
subplot(1,3,1); plot(H(1,:));
subplot(1,3,2); plot(H(2,:));
subplot(1,3,3); plot(H(3,:));

pointL = [0.4716;0.315;1.04];
figure(3)
hold on 
grid on
plot3(H(1,:),H(2,:),H(3,:))
plot3(pointL(1,1),pointL(2,1),pointL(3,1),'o')

%%
q = eps*ones(1,7);
q1 = q(1);
q2 = q(2);
q3 = q(3);
q4 = q(4);
q5 = q(5);
q6 = q(6);
q7 = q(7);
hq = eval(h);
hd = [0.5;0.5;0.5];
hdp = 0;
k1 = [5;5;5];
k2 = [1;1;1];
lamda = 0.1;
pointL = [0.4716;0.315;1.04];

H(:,1) = hq;
eh(:,1) = [0;0;0];
Q(1,:) = q;

%%
damp =1;
kd = 200;
i = 2;
m = 1;
dt = 0.1;
tau = 7;
for i=2:1:100
    q1 = q(1);
    q2 = q(2);
    q3 = q(3);
    q4 = q(4);
    q5 = q(5);
    q6 = q(6);
    q7 = q(7);
    hq = eval(h);
    H(:,i) = hq; 
    eh(:,i) = hd - hq;
    hv = k1 .* tanh(k2 .* eh(:,i));
    JA1 = eval(JA);
    JA1 = JA1';
    J1 = JA1.' * JA1 + lamda*eye(3);
    J2 = inv(J1);
    J = J2 * JA1.' ;
    s = hdp + hv;
    %Jinv = pinv(JA1)



    %Force
    d = pointL - hq;
    v = H(:,i) - H(:,i-1);
    ur = d/norm(d);
    Fd = -damp*(v'*ur)*ur;
    Fk = -kd*d;
    dobsgoal = hd - pointL;
    dactualgoal = hq - hd;
    plan = cross(dactualgoal, dobsgoal);
    plan = plan/ norm(plan);
    force = cross(plan, Fk );
    Ft = Fk + force;
    a = Ft/m;
    vf = v + a *dt;
    
    if norm(d)< 0.3 
        C = J.' * ((1- exp(-tau*norm(d)))*s +  exp(-tau*norm(d))*vf);
     %   norm(d)
    else
        C = J.' *s ;

    end
        q = q + dt*C.';
        Q(i,:) = q;    
end



pointL = [0.4716;0.315;1.04];
figure(3) 
hold on 
grid on
plot3(H(1,:),H(2,:),H(3,:))
plot3(pointL(1,1),pointL(2,1),pointL(3,1),'o')

figure(1)
subplot(1,3,1); plot(eh(1,:));
subplot(1,3,2); plot(eh(2,:));
subplot(1,3,3); plot(eh(3,:));

figure(2)
subplot(1,3,1); plot(H(1,:));
subplot(1,3,2); plot(H(2,:));
subplot(1,3,3); plot(H(3,:));


%%

%% simulation
close all

cupHeight = 0.2;
cupRadius = 0.05;
cupPosition = [0.5, 0.5, cupHeight/2];

lbr = importrobot('iiwa7.urdf'); % 14 kg payload version
lbr.DataFormat = 'row';
gripper = 'iiwa_link_ee_kuka';

body = robotics.RigidBody('cupFrame');
setFixedTransform(body.Joint, trvec2tform(cupPosition))
addBody(lbr, body, lbr.BaseName);

figure;
show(lbr, Q(1,:), 'PreservePlot', false);
hold on
exampleHelperPlotCupAndTable(cupHeight, cupRadius, cupPosition);
plot3(pointL(1,1),pointL(2,1),pointL(3,1),'o')
%%
framerate = 15;
r = robotics.Rate(framerate);
hold on
for k = 1:size(Q,1)
    show(lbr, Q(k,:), 'PreservePlot', false);
    waitfor(r);

end

plot3(H(1,:),H(2,:),H(3,:))
hold off