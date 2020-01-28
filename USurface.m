clc
close all
clear all

load('Kinematics.mat')
Vmax = [ 98 98 100 130 140 180 180] *2*pi/360;

%%
theta = linspace(0,pi,180);
phi = linspace(0,pi,10);
r = 0.1;
xo = 0.476;
yo = 0.315;
zo = 1.04;

for i=1:1: length(theta)
    for j=1:1: length(phi)
        x((i-1)*length(phi) + j,1) = r * cos(theta(i))*sin(phi(j)) + xo;
        y((i-1)*length(phi) + j,1) = r * sin(theta(i))*sin(phi(j)) + yo;
        z((i-1)*length(phi) + j,1) = r * cos(phi(j)) + zo;
    end
    
end


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
kd = 1;
i = 2;
m = 1;
dt = 0.01;
tau = 0.1;
do = 0.1;

for i=2:1:500
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

    Cc = zeros(3,1);
    Cf = zeros(3,1);
    pointLa = [x.';y.';z.'];
    distances = pointLa - hq;
    abd = sqrt(distances(1,:).^2 + distances(2,:).^2  + distances(3,:).^2 ) ;
    nmax = find(abd == min(abd));
    
   for avi = 1:1: length(nmax)
        
        pointL = [x(nmax(avi));y(nmax(avi));z(nmax(avi))];
        %Force
        d = pointL - hq;
        v = H(:,i) - H(:,i-1);
        ur = d/norm(d);
        Fd = -damp*(v'*ur)*ur;
        Fk = -kd*ur*(1/(norm(d)^2) );


        dobsgoal = hd - pointL;
        dactualgoal = hq - hd;
        plan = cross(dactualgoal, dobsgoal);
        plan = plan/ norm(plan);
        force = cross( plan, Fk );
        Ft = force + Fk ;

        Fk1(:,i-1) = Fk/norm(Fk);
        Ft1(:,i-1) = Ft/norm(Ft);
        d1(:,i-1) = d/norm(d);
        da(:,i-1) = dactualgoal / norm(dactualgoal);
        a = Ft/m;
        vf = v + a *dt;

        angle(i-1) = acos(dot(  d , -dactualgoal ) / (norm(d)*norm(dactualgoal)));

        if norm(d)< do
            if angle(i-1) < 90*pi/180
               Cc = Cc +  ((1- exp(-tau*norm(d)))*s );
               Cf = Cf +  ( exp(-tau*norm(d))*vf);
            else
                Cc = Cc + s;
            end
        else
            Cc = Cc + s;
        end

    
    end
        C = J.'*(Cc + Cf);
        for ia = 1:1:7
            if C(ia,1) >= Vmax(ia)
                C(ia,1) = Vmax(ia);
            end
            if C(ia,1) <= -Vmax(ia)
                C(ia,1) = -Vmax(ia);
            end
            
        end
    
       q = q + dt*C.';
        Q(i,:) = q;    
end

%%

pointL = [0.4716;0.315;1.04];
figure(1) 
hold on 
grid on
plot3(H(1,:),H(2,:),H(3,:))
plot3(x,y,z,'o')

figure(2)
subplot(1,3,1); plot(eh(1,:));
subplot(1,3,2); plot(eh(2,:));
subplot(1,3,3); plot(eh(3,:));

figure(3)
subplot(1,3,1); plot(H(1,:));
subplot(1,3,2); plot(H(2,:));
subplot(1,3,3); plot(H(3,:));
