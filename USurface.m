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
kd = 10;
i = 2;
m = 1;
dt = 0.01;
tau = 0.1;
do = 0.1;
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

    pointLa = [x.';y.';z.'];
    distances = pointLa - hq;
    abd = sqrt(distances(1,:).^2 + distances(2,:).^2  + distances(3,:).^2 ) ;
    nmin = find(abd == min(abd));
   
    if abd(nmin(1)) > do
        % collision
        if obstacle == 0

           Obs = [x(min(1));y(min(1));z(nmin(1))];
           Por = Obs - hq;
           Pgr = hd - hq;
           Ang = acos(dot(  Por , Pgr ) / (norm(Por)*norm(Pgr)));
           
           if Ang < 120*pi/180
              %new collision
               obstacle = 1;
               Po = Obs;
               nrg = (hd - hq)/ norm(hd - hq);
               nro = (Po - hq)/norm(Po - hq);
               a = cross(nro,nrg);
               b = cross(a , nro);
            
               v = (H(:,i) - H(:,i-1))/dt ;
               ur = Por/norm(Por);
               Fd = -damp*(v'*ur)*ur;
               Fk = -kd*ur*(1/(norm(Por)^2) )
               Ft = b * norm(Fk) + Fk;
               a = Ft/m;
               vf = v + a *dt;
               Ca = vf;
               
               ba = b;
               Poa = Po;
               
           else
               
               % no collision

           end
          
            
        else
           
            
           for i_aux = 1:1:length(nmin)


               Obs = [x(nmin((i_aux)));y(nmin((i_aux)));z(nmin((i_aux)))];
               Por = Obs - hq;
               Pgr = hd - hq;
               Ang = acos(dot(  Por , Pgr ) / (norm(Por)*norm(Pgr)));


               if Ang < 120*pi/180
                  %new collision
                   obstacle = 1;
                   Po = Obs;
                   nrg = (hd - hq)/ norm(hd - hq);
                   nro = (Po - hq)/norm(Po - hq);
                   a = cross(nro,nrg);
                   b = cross(a , nro);
                   Angb = acos(dot(  b , ba ) / (norm(b)*norm(ba)));
                   
                   if Angb > 90*pi/180
                         b = cross(nro , a );
                   end
        
                   v = (H(:,i) - H(:,i-1))/dt ;
                   ur = Por/norm(Por);
                   Fd = -damp*(v'*ur)*ur;
                   Fk = -kd*ur*(1/(norm(Por)^2) )
                   Ft = b * norm(Fk) + Fk;
                   a = Ft/m;
                   vf = v + a *dt;
                   Ca = Ca + vf;

                   ba = b;
                   Poa = Po;
               end
           end  
           
        end
    else
        % no colision 
        
        if obstacle == 1
           obstacle = 0; 
        end
    end
    
    

    
   
        C = J.'*(Ct + Ca);
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
