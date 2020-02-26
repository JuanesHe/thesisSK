import time
import sys
import urx
import numpy as np
import keyboard
import json
import numpy as np
from numpy.linalg import inv
import triad_openvr
import sys


interval = True

xo = 0.012
yo = 0.01
zo = 0.211
x = 0.05254006385803223 
y = 1.0358047485351562 
z = 1.0186753273010254

v = triad_openvr.triad_openvr()

try:
    rob = urx.Robot("192.168.88.249")
except:
    print('robot error')
    interval = False


if interval:

    #Controller
    dt = 0.1


    print('1')
    P = np.array([0.2,0.2,0.2]).reshape(3,1);
    P_obs = np.array([0.2,0.2,0.2]).reshape(3,1);
    V = np.array([0.0,0.0,0.0]).reshape(3,1)
    Po = np.array([2.1,2.1,10.1]).reshape(3,1);
    e = np.zeros((3,1));
    ea = np.zeros((3,1));
    lamda = 0.01
    k2 = (np.diag([1.2,2.3,3.3]));
    k1 = ( np.diag([1.1,2.3,1.3]));
    k3 = ( np.diag([1.1,2.3,1.3]));

        
    while True:
        tic = time.clock()
        
        try:
            with open('data2.json', 'r') as fa:
                data = json.load(fa)
                k1[0,0] = data['controller']['k11']
                k1[1,1] = data['controller']['k12']
                k1[2,2] = data['controller']['k13']
                k2[0,0] = data['controller']['k21']
                k2[1,1] = data['controller']['k22']
                k2[2,2] = data['controller']['k23']
                k3[0,0] = data['controller']['k31']
                k3[1,1] = data['controller']['k32']
                k3[2,2] = data['controller']['k33']
                vmax = data['controller']['vmax']
                x_refr = data['poss']['xref']
                y_refr = data['poss']['yref']
                z_refr = data['poss']['zref']
                dmin = data['poss_obs']['dmin']
                kd1 = data['poss_obs']['kd1']
                kd2 = data['poss_obs']['kd2']
                kd3 = data['poss_obs']['kd3']
                tau = data['poss_obs']['tau']
        except:
            print('file error')

        try:
            [x1,y1,z1,yaw1,pitch1,roll1] = v.devices["tracker_1"].get_pose_euler();
            pos_x = x1 - x 
            pos_y = y1 - y 
            pos_z = z1 - z 

            print('x:', pos_x,'y:', pos_y,'z:', pos_z)
            xo_refr = -pos_z + xo
            yo_refr = -pos_x + yo
            zo_refr = pos_y + zo 
            print('x_robot:', xo_refr,'y_robot:', yo_refr,'z_robot:', zo_refr)
        except:
            xo_refr = 0
            yo_refr = 0
            zo_refr = 0 

        t = rob.get_pose()
      #  print("Transformation from base to tcp is: ", t)


        pose = rob.getl()
        xn = float(pose[0])
        yn = float(pose[1])
        zn = float(pose[2])
        
        P = np.zeros((3,1));
        P[0,0] = float(xn)
        P[1,0] = float(yn)
        P[2,0] = float(zn)
    #    print('Actual Point',P)
        
        Pa = np.zeros((3,1));   
        Pa[0,0] = float(x_refr)
        Pa[1,0] = float(y_refr)
        Pa[2,0] = float(z_refr)
     #   print('Desired Point',Po)
        
        P_obs = np.zeros((3,1));   
        P_obs[0,0] = float(xo_refr)
        P_obs[1,0] = float(yo_refr)
        P_obs[2,0] = float(zo_refr)
     #   print('Obstacle point',P_obs)
    
        e = Pa - P      
        
        hv = np.matmul(k2,np.tanh(np.matmul(k1,e) + np.matmul(k3,e-ea)))
        ea = e
        
        r_ao = P_obs - P
        n_r_ao = np.linalg.norm(r_ao)
        
        if n_r_ao < dmin:
            #First collision condition
            print('collision condition 1')
            print('e')
            ang = np.arccos(np.matmul(V.transpose(),r_ao)/ (np.linalg.norm(V)*np.linalg.norm(r_ao)) )
            
            if ang < (90)*np.pi/180:
                #seocnd collision condition
                nr = P / np.linalg.norm(P)
                print('collision condition 2')
                nrg = e / np.linalg.norm(e)
                #print(np.linalg.norm(e))
                nv = V / np.linalg.norm(V)
                #print('nv',np.linalg.norm(nv),nv)
                nro = r_ao / np.linalg.norm(r_ao)
                #print('nro',np.linalg.norm(nro),nro)
                a = np.cross(nro.flatten(),nv.flatten())
                #print('a',np.linalg.norm(a),a)
                b = np.cross(a.flatten(),nro.flatten())
                b = b /np.linalg.norm(b)
                #print('b',np.linalg.norm(b),b)
                P_aux = np.zeros((3,1)) 
                P_aux[0,0] = -float(xn)
                P_aux[1,0] = -float(yn) 
                ang_aux = np.arccos(np.matmul(b,nr)/ (np.linalg.norm(nr)*np.linalg.norm(b)) )
                if ang_aux > np.pi/2 :
                    b = b
                else :
                    b = -b
                a1 = np.cross(nro.flatten(),P_aux.flatten())
                #print('a',np.linalg.norm(a1))
                b1 = np.cross(a1.flatten(),nro.flatten())
                #print('b',np.linalg.norm(b1))
                b1 = b1 /np.linalg.norm(b1)
                ang_aux = np.arccos(np.matmul(b1,nr)/ (np.linalg.norm(nr)*np.linalg.norm(b1)) )
                if ang_aux > np.pi/2 :
                    b1 = b1
                else :
                    b1 = -b1
                ur = r_ao/np.linalg.norm(r_ao)
                Fk = (1/(np.power(n_r_ao,3)))
                Fd = np.matmul(V.reshape([1,3]),ur.reshape([3,1]))
                Ft = kd1*b.reshape([3,1])*Fk + kd2*b1.reshape([3,1])*Fk - kd3*Fk*ur.reshape([3,1]) 
                #print('Ft',Ft)
                #print('b',b)
                #print('b1',b1)
                #print('ur',ur)
                #print('hv',hv)
                vx1 = np.exp(-dmin*tau)*Ft[0,0] + (1-np.exp(-dmin*tau))*hv[0,0]
                vy1 = np.exp(-dmin*tau)*Ft[1,0] + (1-np.exp(-dmin*tau))*hv[1,0]
                vz1 = np.exp(-dmin*tau)*Ft[2,0] + (1-np.exp(-dmin*tau))*hv[2,0]
            else:                  
                vx1 = hv[0,0]
                vy1 = hv[1,0]
                vz1 = hv[2,0]
        else:
            vx1 = hv[0,0]
            vy1 = hv[1,0]
            vz1 = hv[2,0]
        
        
        
        if (vx1 > vmax):
            vx1 = vmax;
        elif (vx1 < -vmax):
            vx1 = -vmax;

        if (vy1 > vmax):
            vy1 = vmax;
        elif (vy1 < -vmax):
            vy1 = -vmax;

        if (vz1 > vmax):
            vz1 = vmax;
        elif (vz1 < -vmax):
            vz1 = -vmax;
        



        V[0,0] = vx1
        V[1,0] = vy1
        V[2,0] = vz1


        try:
            rob.speedl((vx1 , vy1, vz1 , 0, 0, 0), acc=0.3, min_time=3);
        except:
            print('sent')

        time.sleep(0)

        toc = time.clock()
        print(e)

else:
    print('finishing')
    try:
        rob.stopl()
    except:
        print('No')