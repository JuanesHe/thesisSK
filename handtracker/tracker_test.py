import triad_openvr
import time
import sys
import urx

#calibration values
xo = 0.012
yo = 0.01
zo = 0.211

rob = urx.Robot("192.168.88.249")

v = triad_openvr.triad_openvr()
v.print_discovered_objects()

if len(sys.argv) == 1:
    interval = 1/250
elif len(sys.argv) == 2:
    interval = 1/float(sys.argv[1])
else:
    print("Invalid number of arguments")
    interval = False
    
if interval:
    for i in range(0,1):
        start = time.time()
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
        rob.close()

