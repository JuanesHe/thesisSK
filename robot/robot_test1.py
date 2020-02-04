import triad_openvr
import time
import sys
import urx
import numpy as np
import keyboard

#calibration values
xo = -0.007
yo = 0
zo = 0.217



v = triad_openvr.triad_openvr()
v.print_discovered_objects()




if len(sys.argv) == 1:
    interval = 1/250
elif len(sys.argv) == 2:
    interval = 1/float(sys.argv[1])
else:
    print("Invalid number of arguments")
    interval = False

try:
    rob = urx.Robot("192.168.88.249")
except:
    print('robot error')
    interval = False

try:
    [x,y,z,yaw,pitch,roll] = v.devices["tracker_1"].get_pose_euler()
    [x1,y1,z1,yaw1,pitch1,roll1] = v.devices["tracker_2"].get_pose_euler()
except:
    print('tracker error')
    interval = False



if interval:


    #Controller

    r = 10
    phi = np.linspace(0,2*np.pi,9)
    teta = np.linspace(0,np.pi,5)

    sint =  np.sin(teta)
    sint=np.asmatrix(sint)
    sint= sint.transpose(1,0)
    cosp = np.cos(phi)
    cosp=np.asmatrix(cosp)
    x = r * sint*cosp;
    x = np.array(x.reshape(45,1))

    sint =  np.sin(teta)
    sint=np.asmatrix(sint)
    sint= sint.transpose(1,0)
    sinp = np.sin(phi)
    sinp=np.asmatrix(sinp)
    y = r * sint*sinp;
    y = np.array(y.reshape(45,1))

    cost =  np.cos(teta)
    cost = np.asmatrix(cost)
    cost = cost.transpose(1,0)
    cosp = np.cos(phi*0)
    cosp = np.asmatrix(cosp)
    z = r * cost*cosp;
    z = np.array(z.reshape(45,1))


    T = 0.1
    k = 1;
    m = 5;
    d = 2;
    P = np.array([0,0,0]).reshape(3,1);
    rno = np.concatenate((x, y, z ), axis=0).reshape(3,45)
    V = np.array([0,0,0]).reshape(3,1)
    Po = np.array([2,2,10]).reshape(3,1);
    ro = np.array([2,2,0]).reshape(3,1);
    rn = rno + Po

    while not keyboard.is_pressed('q'):
        try:
            [x,y,z,yaw,pitch,roll] = v.devices["tracker_1"].get_pose_euler();
            [x1,y1,z1,yaw1,pitch1,roll1] = v.devices["tracker_2"].get_pose_euler();
            pos_x = x1 - x 
            pos_y = y1 - y 
            pos_z = z1 - z 
            
            print('x:', pos_x,'y:', pos_y,'z:', pos_z)
            x_refr = -pos_z + xo
            y_refr = -pos_x + yo
            z_refr = pos_y + zo 
            print('x_robot:', x_refr,'y_robot:', y_refr,'z_robot:', z_refr)

            t = rob.get_pose()
            print("Transformation from base to tcp is: ", t)


            pose = rob.getl()
            xn = float(pose[0])
            yn = float(pose[1])
            zn = float(pose[2])
            
            print("Transformation from base to tcp is: ", xn)
            xn = float(pose[0])
            yn = float(pose[1])
            zn = float(pose[2])
            P = np.zeros((3,1));
            P[0,0] = float(xn)
            P[1,0] = float(yn)
            P[2,0] = float(zn)
            
            print('Punto actual',P)
            Po = np.zeros((3,1));   
            Po[0,0] = float(x_refr)
            if (Po[0,0] > -0.27):
                Po[0,0] = -0.3
            if (Po[0,0] < -0.44):
                Po[0,0] = -0.44
                
            Po[1,0] = float(y_refr)
            if (Po[1,0] > 0.25):
                Po[1,0] = 0.25
            if (Po[1,0] < -0.25):
                Po[1,0] = -0.25
                
                
            Po[2,0] = float(z_refr)
            if (Po[2,0] > 0.40):
                Po[2,0] = 0.40
            if (Po[2,0] < 0.2):
                Po[2,0] = 0.2
                
                
            print('punto deseado',Po)
            rn = rno + Po
            rf = P - rn 
            dr = P - Po
            
            mrf = np.sqrt(np.power(rf[0,:],2) + np.power(rf[1,:],2) + np.power(rf[2,:],2) );
            urf = rf/mrf
            Fk = (-k)*urf*(np.dot(dr.reshape(1,3),urf))
            Fd = (-d)*urf*(np.dot(V.reshape(1,3),urf))
            a = np.sum(Fk+Fd, axis=1) /m
            a.resize(3,1)
            V = a*T + V    
            print ('velocidad',V)          

            
            vx1 = V[0,0]
            vy1 = V[1,0]
            vz1 = V[2,0]
            
            if (vx1>0.2):
                vx1=0.2;
            elif (vx1<-0.2):
                vx1=-0.2;

            if (vy1>0.2):
                vy1=0.2;
            elif (vy1<-0.2):
                vy1=-0.2;

            if (vz1>0.2):

                vz1=0.2;
            elif (vz1<-0.2):
                vz1=-0.2;
            
            
            time.sleep(0.1)
            
            rob.speedl((vx1 , vy1, vz1 , 0, 0, 0), acc=0.3, min_time=3);
        except:
            rob.close()

        

