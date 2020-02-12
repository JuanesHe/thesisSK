import triad_openvr
import time
import sys
import urx
import numpy as np
import keyboard
import json
import numpy as np
from numpy.linalg import inv

def Jacobian(q1,q2,q3,q4,q5,q6):
    JA11 = (float) (2247*np.cos(q1))/20000 + (819*np.cos(q1)*np.cos(q5))/10000 + (4873*np.cos(q2)*np.sin(q1))/20000 - (1707*np.cos(q4)*(np.cos(q2)*np.sin(q1)*np.sin(q3) + np.cos(q3)*np.sin(q1)*np.sin(q2)))/20000 + (1707*np.sin(q4)*(np.sin(q1)*np.sin(q2)*np.sin(q3) - np.cos(q2)*np.cos(q3)*np.sin(q1)))/20000 - (819*np.sin(q5)*(np.cos(q4)*(np.sin(q1)*np.sin(q2)*np.sin(q3) - np.cos(q2)*np.cos(q3)*np.sin(q1)) + np.sin(q4)*(np.cos(q2)*np.sin(q1)*np.sin(q3) + np.cos(q3)*np.sin(q1)*np.sin(q2))))/10000 - (853*np.sin(q1)*np.sin(q2)*np.sin(q3))/4000 + (853*np.cos(q2)*np.cos(q3)*np.sin(q1))/4000
    JA12 = float (4873*np.cos(q1)*np.sin(q2))/20000 + (1707*np.cos(q4)*(np.cos(q1)*np.cos(q2)*np.cos(q3) - np.cos(q1)*np.sin(q2)*np.sin(q3)))/20000 - (1707*np.sin(q4)*(np.cos(q1)*np.cos(q2)*np.sin(q3) + np.cos(q1)*np.cos(q3)*np.sin(q2)))/20000 + (819*np.sin(q5)*(np.cos(q4)*(np.cos(q1)*np.cos(q2)*np.sin(q3) + np.cos(q1)*np.cos(q3)*np.sin(q2)) + np.sin(q4)*(np.cos(q1)*np.cos(q2)*np.cos(q3) - np.cos(q1)*np.sin(q2)*np.sin(q3))))/10000 + (853*np.cos(q1)*np.cos(q2)*np.sin(q3))/4000 + (853*np.cos(q1)*np.cos(q3)*np.sin(q2))/4000
    JA13 = float (1707*np.cos(q4)*(np.cos(q1)*np.cos(q2)*np.cos(q3) - np.cos(q1)*np.sin(q2)*np.sin(q3)))/20000 - (1707*np.sin(q4)*(np.cos(q1)*np.cos(q2)*np.sin(q3) + np.cos(q1)*np.cos(q3)*np.sin(q2)))/20000 + (819*np.sin(q5)*(np.cos(q4)*(np.cos(q1)*np.cos(q2)*np.sin(q3) + np.cos(q1)*np.cos(q3)*np.sin(q2)) + np.sin(q4)*(np.cos(q1)*np.cos(q2)*np.cos(q3) - np.cos(q1)*np.sin(q2)*np.sin(q3))))/10000 + (853*np.cos(q1)*np.cos(q2)*np.sin(q3))/4000 + (853*np.cos(q1)*np.cos(q3)*np.sin(q2))/4000
    JA14 = float (1707*np.cos(q4)*(np.cos(q1)*np.cos(q2)*np.cos(q3) - np.cos(q1)*np.sin(q2)*np.sin(q3)))/20000 - (1707*np.sin(q4)*(np.cos(q1)*np.cos(q2)*np.sin(q3) + np.cos(q1)*np.cos(q3)*np.sin(q2)))/20000 + (819*np.sin(q5)*(np.cos(q4)*(np.cos(q1)*np.cos(q2)*np.sin(q3) + np.cos(q1)*np.cos(q3)*np.sin(q2)) + np.sin(q4)*(np.cos(q1)*np.cos(q2)*np.cos(q3) - np.cos(q1)*np.sin(q2)*np.sin(q3))))/10000
    JA15 = float (- (819*np.sin(q1)*np.sin(q5))/10000 - (819*np.cos(q5)*(np.cos(q4)*(np.cos(q1)*np.cos(q2)*np.cos(q3) - np.cos(q1)*np.sin(q2)*np.sin(q3)) - np.sin(q4)*(np.cos(q1)*np.cos(q2)*np.sin(q3) + np.cos(q1)*np.cos(q3)*np.sin(q2))))/10000)
    JA16 = 0
    JA21 = float (2247*np.sin(q1))/20000 - (4873*np.cos(q1)*np.cos(q2))/20000 + (819*np.cos(q5)*np.sin(q1))/10000 + (1707*np.cos(q4)*(np.cos(q1)*np.cos(q2)*np.sin(q3) + np.cos(q1)*np.cos(q3)*np.sin(q2)))/20000 + (1707*np.sin(q4)*(np.cos(q1)*np.cos(q2)*np.cos(q3) - np.cos(q1)*np.sin(q2)*np.sin(q3)))/20000 - (819*np.sin(q5)*(np.cos(q4)*(np.cos(q1)*np.cos(q2)*np.cos(q3) - np.cos(q1)*np.sin(q2)*np.sin(q3)) - np.sin(q4)*(np.cos(q1)*np.cos(q2)*np.sin(q3) + np.cos(q1)*np.cos(q3)*np.sin(q2))))/10000 - (853*np.cos(q1)*np.cos(q2)*np.cos(q3))/4000 + (853*np.cos(q1)*np.sin(q2)*np.sin(q3))/4000
    JA22 = float (4873*np.sin(q1)*np.sin(q2))/20000 - (1707*np.cos(q4)*(np.sin(q1)*np.sin(q2)*np.sin(q3) - np.cos(q2)*np.cos(q3)*np.sin(q1)))/20000 - (1707*np.sin(q4)*(np.cos(q2)*np.sin(q1)*np.sin(q3) + np.cos(q3)*np.sin(q1)*np.sin(q2)))/20000 + (819*np.sin(q5)*(np.cos(q4)*(np.cos(q2)*np.sin(q1)*np.sin(q3) + np.cos(q3)*np.sin(q1)*np.sin(q2)) - np.sin(q4)*(np.sin(q1)*np.sin(q2)*np.sin(q3) - np.cos(q2)*np.cos(q3)*np.sin(q1))))/10000 + (853*np.cos(q2)*np.sin(q1)*np.sin(q3))/4000 + (853*np.cos(q3)*np.sin(q1)*np.sin(q2))/4000
    JA23 = float (819*np.sin(q5)*(np.cos(q4)*(np.cos(q2)*np.sin(q1)*np.sin(q3) + np.cos(q3)*np.sin(q1)*np.sin(q2)) - np.sin(q4)*(np.sin(q1)*np.sin(q2)*np.sin(q3) - np.cos(q2)*np.cos(q3)*np.sin(q1))))/10000 - (1707*np.sin(q4)*(np.cos(q2)*np.sin(q1)*np.sin(q3) + np.cos(q3)*np.sin(q1)*np.sin(q2)))/20000 - (1707*np.cos(q4)*(np.sin(q1)*np.sin(q2)*np.sin(q3) - np.cos(q2)*np.cos(q3)*np.sin(q1)))/20000 + (853*np.cos(q2)*np.sin(q1)*np.sin(q3))/4000 + (853*np.cos(q3)*np.sin(q1)*np.sin(q2))/4000
    JA24 = float (819*np.sin(q5)*(np.cos(q4)*(np.cos(q2)*np.sin(q1)*np.sin(q3) + np.cos(q3)*np.sin(q1)*np.sin(q2)) - np.sin(q4)*(np.sin(q1)*np.sin(q2)*np.sin(q3) - np.cos(q2)*np.cos(q3)*np.sin(q1))))/10000 - (1707*np.sin(q4)*(np.cos(q2)*np.sin(q1)*np.sin(q3) + np.cos(q3)*np.sin(q1)*np.sin(q2)))/20000 - (1707*np.cos(q4)*(np.sin(q1)*np.sin(q2)*np.sin(q3) - np.cos(q2)*np.cos(q3)*np.sin(q1)))/20000
    JA25 = float (819*np.cos(q1)*np.sin(q5))/10000 + (819*np.cos(q5)*(np.cos(q4)*(np.sin(q1)*np.sin(q2)*np.sin(q3) - np.cos(q2)*np.cos(q3)*np.sin(q1)) + np.sin(q4)*(np.cos(q2)*np.sin(q1)*np.sin(q3) + np.cos(q3)*np.sin(q1)*np.sin(q2))))/10000
    JA26 = 0
    JA31 = 0
    JA32 = float (853*np.sin(q2)*np.sin(q3))/4000 - (853*np.cos(q2)*np.cos(q3))/4000 - (4873*np.cos(q2))/20000 - (819*np.sin(q5)*(np.cos(q4)*(np.cos(q2)*np.cos(q3) - np.sin(q2)*np.sin(q3)) - np.sin(q4)*(np.cos(q2)*np.sin(q3) + np.cos(q3)*np.sin(q2))))/10000 + (1707*np.cos(q4)*(np.cos(q2)*np.sin(q3) + np.cos(q3)*np.sin(q2)))/20000 + (1707*np.sin(q4)*(np.cos(q2)*np.cos(q3) - np.sin(q2)*np.sin(q3)))/20000
    JA33 = float (853*np.sin(q2)*np.sin(q3))/4000 - (853*np.cos(q2)*np.cos(q3))/4000 - (819*np.sin(q5)*(np.cos(q4)*(np.cos(q2)*np.cos(q3) - np.sin(q2)*np.sin(q3)) - np.sin(q4)*(np.cos(q2)*np.sin(q3) + np.cos(q3)*np.sin(q2))))/10000 + (1707*np.cos(q4)*(np.cos(q2)*np.sin(q3) + np.cos(q3)*np.sin(q2)))/20000 + (1707*np.sin(q4)*(np.cos(q2)*np.cos(q3) - np.sin(q2)*np.sin(q3)))/20000
    JA34 = float (1707*np.cos(q4)*(np.cos(q2)*np.sin(q3) + np.cos(q3)*np.sin(q2)))/20000 - (819*np.sin(q5)*(np.cos(q4)*(np.cos(q2)*np.cos(q3) - np.sin(q2)*np.sin(q3)) - np.sin(q4)*(np.cos(q2)*np.sin(q3) + np.cos(q3)*np.sin(q2))))/10000 + (1707*np.sin(q4)*(np.cos(q2)*np.cos(q3) - np.sin(q2)*np.sin(q3)))/20000
    JA35 = float (-(819*np.cos(q5)*(np.cos(q4)*(np.cos(q2)*np.sin(q3) + np.cos(q3)*np.sin(q2)) + np.sin(q4)*(np.cos(q2)*np.cos(q3) - np.sin(q2)*np.sin(q3))))/10000)
    JA36 = 0
    JA = np.matrix([[JA11,JA12,JA13,JA14,JA15,JA16],[JA21,JA22,JA23,JA24,JA25,JA26],[JA31,JA32,JA33,JA34,JA35,JA36]])
    return (JA)

def pos_rob(q1,q2,q3,q4,q5,q6):
    x = (2247*np.sin(q1))/20000 - (4873*np.cos(q1)*np.cos(q2))/20000 + (819*np.cos(q5)*np.sin(q1))/10000 + (1707*np.cos(q4)*(np.cos(q1)*np.cos(q2)*np.sin(q3) + np.cos(q1)*np.cos(q3)*np.sin(q2)))/20000 + (1707*np.sin(q4)*(np.cos(q1)*np.cos(q2)*np.cos(q3) - np.cos(q1)*np.sin(q2)*np.sin(q3)))/20000 - (819*np.sin(q5)*(np.cos(q4)*(np.cos(q1)*np.cos(q2)*np.cos(q3) - np.cos(q1)*np.sin(q2)*np.sin(q3)) - np.sin(q4)*(np.cos(q1)*np.cos(q2)*np.sin(q3) + np.cos(q1)*np.cos(q3)*np.sin(q2))))/10000 - (853*np.cos(q1)*np.cos(q2)*np.cos(q3))/4000 + (853*np.cos(q1)*np.sin(q2)*np.sin(q3))/4000
    y = (1707*np.cos(q4)*(np.cos(q2)*np.sin(q1)*np.sin(q3) + np.cos(q3)*np.sin(q1)*np.sin(q2)))/20000 - (819*np.cos(q1)*np.cos(q5))/10000 - (4873*np.cos(q2)*np.sin(q1))/20000 - (2247*np.cos(q1))/20000 - (1707*np.sin(q4)*(np.sin(q1)*np.sin(q2)*np.sin(q3) - np.cos(q2)*np.cos(q3)*np.sin(q1)))/20000 + (819*np.sin(q5)*(np.cos(q4)*(np.sin(q1)*np.sin(q2)*np.sin(q3) - np.cos(q2)*np.cos(q3)*np.sin(q1)) + np.sin(q4)*(np.cos(q2)*np.sin(q1)*np.sin(q3) + np.cos(q3)*np.sin(q1)*np.sin(q2))))/10000 + (853*np.sin(q1)*np.sin(q2)*np.sin(q3))/4000 - (853*np.cos(q2)*np.cos(q3)*np.sin(q1))/4000
    z = (1707*np.sin(q4)*(np.cos(q2)*np.sin(q3) + np.cos(q3)*np.sin(q2)))/20000 - (853*np.cos(q2)*np.sin(q3))/4000 - (853*np.cos(q3)*np.sin(q2))/4000 - (819*np.sin(q5)*(np.cos(q4)*(np.cos(q2)*np.sin(q3) + np.cos(q3)*np.sin(q2)) + np.sin(q4)*(np.cos(q2)*np.cos(q3) - np.sin(q2)*np.sin(q3))))/10000 - (1707*np.cos(q4)*(np.cos(q2)*np.cos(q3) - np.sin(q2)*np.sin(q3)))/20000 - (4873*np.sin(q2))/20000 + 1519/10000
    h = [float(x),float(y),float(z)]
    return h

#calibration values
xo = -0.007
yo = 0
zo = 0.217

try :
    with open('data.json', 'r') as fa:
        data = json.load(fa)
        T = data['person']['T']
        k = data['person']['k']
        m = data['person']['m']
        d = data['person']['d']
except:
    print('file error')
    interval = False  


#v = triad_openvr.triad_openvr()
#v.print_discovered_objects()




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

#try:
#    [x,y,z,yaw,pitch,roll] = v.devices["tracker_1"].get_pose_euler()
#    [x1,y1,z1,yaw1,pitch1,roll1] = v.devices["tracker_2"].get_pose_euler()
#except:
#    print('tracker error')
#    interval = False



if interval:


    #Controller
    dt = 0.1

    r = 5
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


    print('1')
    P = np.array([0,0,0]).reshape(3,1);
    rno = np.concatenate((x, y, z ), axis=0).reshape(3,45)
    V = np.array([0,0,0]).reshape(3,1)
    Po = np.array([2,2,10]).reshape(3,1);
    ro = np.array([2,2,0]).reshape(3,1);
    rn = rno + Po
    Pa = np.zeros((3,1));
    lamda = 0.01

    while True:
        
        if keyboard.is_pressed('q'):  # if key 'q' is pressed 
            print('You Pressed A Key!')
            break  # finishing the loop

#           [x,y,z,yaw,pitch,roll] = v.devices["tracker_1"].get_pose_euler();
#           [x1,y1,z1,yaw1,pitch1,roll1] = v.devices["tracker_2"].get_pose_euler();
#            pos_x = x1 - x 
#            pos_y = y1 - y 
#            pos_z = z1 - z 
        pos_x = 0.321
        pos_y = 0.1
        pos_z = 0.3

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

        Va = P - Pa 
        Pa = P
        
        print('Punto actual',P)
        Po = np.zeros((3,1));   
        Po[0,0] = float(x_refr)
        if (Po[0,0] < 0.27):
            Po[0,0] = 0.27
        if (Po[0,0] > 0.44):
            Po[0,0] = 0.44
            
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
        
        try :
            with open('data.json', 'r') as fa:
                data = json.load(fa)
                T = data['person']['T']
                k = data['person']['k']
                m = data['person']['m']
                d = data['person']['d']
                vmax = data['person']['vmax']
        except:
            print('file error')


        mrf = np.sqrt(np.power(rf[0,:],2) + np.power(rf[1,:],2) + np.power(rf[2,:],2) );
        urf = rf/mrf
        Fk = (-k)*urf*(np.dot(dr.reshape(1,3),urf))
        Fd = (-d)*urf*(np.dot(V.reshape(1,3),urf))
        a = np.sum(Fk+Fd, axis=1) /m
        a.resize(3,1)
        V = a*T + V    
        #print ('velocidad',V)          

        
        vx1 = V[0,0]
        vy1 = V[1,0]
        vz1 = V[2,0]
        
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

        print ('velocidad',vx1,vy1,vz1) 

        J = rob.getj()
        print(J)
        JA = Jacobian(J[0],J[1],J[2],J[3],J[4],J[5])
        JAp = JA.transpose();
        JA1 = JA * JAp + np.eye(3)*lamda
        J2 = inv(JA1)*JA
        C = J2.transpose()*np.matrix(([[V[0,0],[V[1,0]],[V[2,0]]]))
        
        J = J + dt*C.transpose()
        J = [float(J[0,0]), float(J[0,1]), float(J[0,2]), float(J[0,3]), float(J[0,4]), float(J[0,5])]
        try:
            rob.movej(J,acc=0.02,vel=0.02)
        except:
            print('sent')

        time.sleep(0.1)



else:
    print('finishing')
    try:
        rob.stopl()
    except:
        print('No')
