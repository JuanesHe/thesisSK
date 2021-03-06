clc
close all
clear all

load('Kinematics.mat')
Vmax = 0.6*[ 98 98 100 130 140 180 180] *2*pi/360;

%%
theta = linspace(0*pi/180,180*pi/180,180);
phi = linspace(0,pi,180);
r = 0.2;
xo = -0.476;
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
hd = [-0.5;0.5;0.5];
hdp = 0;
k1 = [5;5;5];
k2 = [1;1;1];
lamda = 0.1;
pointL = [-0.45;0.45;0.45];

H(:,1) = hq;
eh(:,1) = [0;0;0];
Q(1,:) = q;

%%
damp =1;
kd1 = 0.5;
kd2 = 0.4;
kd3 = 0.8;
i = 2;
m = 5;
dt = 0.01;
tau = 1;
do = 0.15;
obstacle = 0;



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
    C = zeros(3,1);
    
    Ca = zeros(3,1);
    Ct = s;

    %pointLa = [x.';y.';z.'];
    pointLa = pointL;
    distances = pointLa - hq;
    abd = sqrt(distances(1,:).^2 + distances(2,:).^2  + distances(3,:).^2 ) ;
    nmin = find( abd < do );
   
    if find( abd < do )
        % collission condition 1
        dmin = abd(find( abd == min(abd) ));
        for cont_obs = 1:1: length(nmin)
           Obs = [x(min(cont_obs));y(min(cont_obs));z(nmin(cont_obs))];
           Por = Obs - hq;
           Pgr = hd - hq;
           nv = (H(:,i) - H(:,i-1))/ norm(H(:,i) - H(:,i-1));
           Ang = acos(dot(  Por , nv ) / (norm(Por)*norm(nv)));
           if Ang < 90*pi/180
              % collision 1
            
               Po = Obs;
               nrg = (hd - hq)/ norm(hd - hq);
               nv = (H(:,i) - H(:,i-1))/ norm(H(:,i) - H(:,i-1));
               nro = (Po - hq)/norm(Po - hq);
               
               a = cross(nro,nv);
               b = cross(a , nro);
               a1 = cross(nro,nrg);
               b1 = cross(a1 , nro);
               
               if obstacle == 1
                   
               
               else
                  obstacle = 0;
               end
            
               v = (H(:,i) - H(:,i-1))/dt ;
               ur = Por/norm(Por);
               Fd = -damp*(v'*ur)*ur;
               Fk = -ur*(1/(norm(Por)^4) );
               Ft = b * kd1* norm(Fk) + kd2* Fk + b1 * kd3*norm(Fk);
               a = Ft/m;
               vf = v + a *dt;
               Ca = vf;
               
               
           else
              % collision 2
              
              
                Po = Obs;
               nrg = (hd - hq)/ norm(hd - hq);
               nv = (H(:,i) - H(:,i-1))/ norm(H(:,i) - H(:,i-1));
               nro = (Po - hq)/norm(Po - hq);
               a = cross(nro,nv);
               b = cross(a , nro);
               a1 = cross(nro,nrg);
               b1 = cross(a1 , nro);
            
               v = (H(:,i) - H(:,i-1))/dt ;
               ur = Por/norm(Por);
               Fd = -damp*(v'*ur)*ur;
               Fk = -ur*(1/(norm(Por)^4) );
               Ft = kd2* Fk ;
               a = Ft/m;
               vf = v + a *dt;
               Ca = vf;
               
               
              
           end
        end
        Ctotal = ((1- exp(-tau*norm(dmin)))*Ct +  exp(-tau*norm(dmin))*Ca);
    else 
        dmin= 0;
        obstacle = 0;
        Ctotal = Ct;
        
    end

    
    


   
        C = J.'*(Ctotal);
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