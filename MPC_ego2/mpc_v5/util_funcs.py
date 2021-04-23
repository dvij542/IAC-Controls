import states
from casadi import *
import params as p
import math
from casadi import *

# Consts
pcy1 = 1.603
pdy1 = 1.654
pdy2 = -0.1783
pdy3 = 0
pey1 = -1.409
pey2 = -1.6617
pey3 = 0.26886
pey4 = -13.61
pky1 = -53.05
pky2 = 4.1265
pky3 = 1.5016
phy1 = 0.0039
phy2 = -0.00253
pvy1 = -0.01038
pvy2 = -0.1
pvy3 = 0.4498
pvy4 = -1.5

rby1 = 35.304
rby2 = 15.666
rby3 = -0.01179

rcy1 = 1.018
rey1 = 0.35475
rey2 = 0.01966
rhy1 = 0.00667
rhy2 = 0.00207

SC_REAR_RIGHT_EDGE = 0
SC_REAR_LEFT_EDGE = 1
SC_FRONT_RIGHT_EDGE = 2
SC_FRONT_LEFT_EDGE = 3
SC_REAR_FACE_CENTER = 4
SC_FRONT_FACE_CENTER = 5
SC_RIGHT_FACE_CENTER = 6
SC_LEFT_FACE_CENTER = 7
SC_UNUSED = 8
SC_UNKNOWN_FRONT_FACE_RIGHT = 9
SC_UNKNOWN_FRONT_FACE_LEFT = 10
SC_UNKNOWN_REAR_FACE_RIGHT = 11
SC_UNKNOWN_REAR_FACE_LEFT = 12
SC_UNKNOWN_RIGHT_FACE_FRONT = 13
SC_UNKNOWN_LEFT_FACE_FRONT = 14
SC_UNKNOWN_RIGHT_FACE_REAR = 15
SC_UNKNOWN_LEFT_FACE_REAR = 16

def sigmoid(x) :
    return if_else(x<0,SX.exp(x)/(SX.exp(x)+1),1/(SX.exp(-x)+1)) 

def inside_region_2(px,py):
    if py < -1144 and px < -240 :
        return True
    return False

def get_center_line(px,py,angle_heading) :
    X1 = -315
    Y1 = -1151
    X2 = -309
    Y2 = -1454
    theta = atan2(Y2-Y1,X2-X1)
    theta_diff = angle_heading - theta
    m = -tan(theta_diff)
    M = (Y2-Y1)/(X2-X1)
    dist = ((py-Y1)-M*(px-X1))/sqrt(1+M**2)
    c = -dist/cos(theta_diff)
    return [c,m,0,0] 

def calc_force_from_slip_angle(slip_angle,fz,fz0)
    epsilon = 0.0001
    dfz = (fz-fz0)/fz0
    shy = phy1 + phy2*dfz
	ky = slip_angle + shy
	Cy = pcy1
	muy = pdy1 + pdy2*dfz
	Dy = muy*fz
	Ey = (pey1 + pey2*dfz + pey3*dfz**2)*(1-pey4)
	K = fz*(pky1+pky2*dfz)*exp(pky3*dfz)
	By=K/(Cy*Dy+epsilon)
	svy = fz*(pvy1+pvy2*dfz+pvy3*dfz**2+pvy4*dfz**3)
	Fy = Dy*sin(Cy*np.atan(By*ky - Ey*(By*ky-np.atan(By*ky))))
    return Fy

def calc_force_from_slip(slip,speed) :
    fz = p.fz0 + p.lift_coeff*speed**2
    dfz = (fz-p.fz0)/p.fz0
    shx = phx1 + phx2*dfz
    kx = slip + shx
    Cx = pcx1
    mux = pdx1 + pdx2*dfz
    Dx = mux*fz
    Ex = (pex1 + pex2*dfz + pex3*dfz**2)*(1-pex4)
    K = (fz+dfz*fz)*(pkx1+pkx2*dfz)*np.exp(pkx3*dfz)
    Bx=K/(Cx*Dx+epsilon)
    svx = fz*(pvx1+pvx2*dfz)
    Fx = Dx*np.sin(Cx*np.arctan(Bx*kx - Ex*(Bx*kx-np.arctan(Bx*kx)))) + svx
    return Fx
    
def get_gear_radii(curr_speed):
    gear_radii = 0
    for i in range(len(p.gear_change_speeds)-1):
        gear_radii = gear_radii + (curr_speed>=p.gear_change_speeds[i])*(curr_speed<p.gear_change_speeds[i+1])*p.gear_radiis[i]
    return gear_radii

def car_speed_to_gear_speed(curr_speed):
    gear_speed = 0
    for i in range(len(p.gear_change_speeds)-1):
        gear_speed = gear_speed + (curr_speed>=p.gear_change_speeds[i])*(curr_speed<p.gear_change_speeds[i+1])*(p.gear_speed_l[i] + (p.gear_speed_r[i]-p.gear_speed_l[i])*(curr_speed-p.gear_change_speeds[i])/(p.gear_change_speeds[i+1]-p.gear_change_speeds[i]))
    return gear_speed*(60/6.28)

def calc_torque_from_gear_speed(gear_speed,curr_c):
    col_1 = [0,500,1500,2500,6300,6600,6900,7200,7500,7800,8100,
        8400,8700,9000,9300,9600,9800]

    row_1 = [0,10,20,30,40,50,60,70,80,90,100]

    eng_data = [[0,1.9,1.9,3.8,5.7,9.5,17.1,22.8,24.7,38,43.7],
                [0,1.9,1.9,3.8,5.7,9.5,17.1,22.8,24.7,38,43.7],
                [0,1.9,3.8,5.7,7.6,13.3,20.9,26.6,30.4,41.8,47.5],
                [-5.7,0,1.9,5.7,7.6,15.2,24.7,30.4,34.2,45.6,49.97],
                [-7.6,0,1.9,7.6,9.5,19,26.6,32.3,36.1,47.5,51.3],
                [-9.5,1.9,3.8,9.5,11.4,20.9,28.5,34.2,38,47.5,51.3],
                [-11.4,3.8,7.6,13.3,15.2,22.8,30.4,36.1,39.9,49.4,51.11],
                [-13.3,1.9,11.4,17.1,19,26.6,34.2,38,43.7,51.3,52.25],
    
                [-17.1,1.9,11.4,17.1,19,26.6,34.2,38,43.7,51.3,52.25],            [-15.2,3.8,15.2,19,20.9,28.5,36.1,39.9,44.65,52.25,53.01],
                [-19,0,7.6,13.3,15.2,22.8,30.4,36.1,39.9,48.45,51.3],
                [-22.8,-3.8,3.8,9.5,11.4,19,28.5,32.3,38,47.5,50.16],
                [-24.7,-5.7,1.9,7.6,9.5,15.2,24.7,28.5,34.2,45.6,49.02],
                [-24.7,-6.65,0,5.7,5.7,13.3,22.8,26.6,32.3,43.7,47.31],
                [-26.6,-7.6,-1.9,1.9,3.8,11.4,20.9,24.7,30.4,41.8,44.65],
                [-27.55,-9.5,-2.85,0,1.9,9.5,19,22.8,28.5,39.9,42.56],
                [-28.5,-10.45,-3.8,-1.9,0,7.6,17.1,20.9,26.6,37.05,40.66]]

    if(gear_speed==9800):
        gear_speed = 9800-0.01
    if(curr_c ==100):
        curr_c =100-0.01

    torque_val = 0
    for indx1 in range(1,len(row_1)):    
        for indx2 in range(1,len(col_1)):    
            x1 = row_1[indx1-1]
            x2 = row_1[indx1]
            y1 = col_1[indx2-1]
            y2 = col_1[indx2]
            q11 = eng_data[indx2-1][indx1-1]
            q12 = eng_data[indx2-1][indx1]
            q21 = eng_data[indx2][indx1-1]
            q22 = eng_data[indx2][indx1]
            x = curr_c
            y = gear_speed
            torque_val = torque_val+(x>=x1)*(x<x2)*(y>=y1)*(y<y2) * \
                    (q11 * (x2 - x) * (y2 - y) + \
                    q21 * (x - x1) * (y2 - y) + \
                    q12 * (x2 - x) * (y - y1) + \
                    q22 * (x - x1) * (y - y1) \
                   ) / ((x2 - x1) * (y2 - y1) + 0.0)
    return torque_val

def dist1(x1,y1,x2,y2):
    return ((x1-x2)**2 + (y1-y2)**2)**(1/2)

def anchorPointToCenter(x,y,t,no) :
    xd = 0
    yd = 0
    if(no==SC_REAR_RIGHT_EDGE) :
        xd = p.L/2
        yd = p.W/2
    if(no==SC_REAR_LEFT_EDGE) :
        xd = p.L/2
        yd = -p.W/2
    if(no==SC_FRONT_RIGHT_EDGE) :
        xd = -p.L/2
        yd = p.W/2
    if(no==SC_FRONT_LEFT_EDGE) :
        xd = -p.L/2
        yd = -p.W/2
    if(no==SC_REAR_FACE_CENTER) :
        xd = p.L/2
        yd = 0
    if(no==SC_FRONT_FACE_CENTER) :
        xd = -p.L/2
        yd = 0
    if(no==SC_RIGHT_FACE_CENTER) :
        xd = 0
        yd = p.W/2
    if(no==SC_LEFT_FACE_CENTER) :
        xd = 0
        yd = -p.W/2
    if(no==SC_UNKNOWN_FRONT_FACE_RIGHT) :
        xd = -p.L/2
        yd = p.W/4
    if(no==SC_UNKNOWN_FRONT_FACE_LEFT) :
        xd = -p.L/2
        yd = -p.W/4
    if(no==SC_UNKNOWN_REAR_FACE_RIGHT) :
        xd = p.L/2
        yd = p.W/4
    if(no==SC_UNKNOWN_REAR_FACE_LEFT) :
        xd = p.L/2
        yd = -p.W/4
    if(no==SC_UNKNOWN_RIGHT_FACE_FRONT) :
        xd = -p.L/4
        yd = p.W/2
    if(no==SC_UNKNOWN_LEFT_FACE_FRONT) :
        xd = -p.L/4
        yd = -p.W/2
    if(no==SC_UNKNOWN_RIGHT_FACE_REAR) :
        xd = p.L/4
        yd = p.W/2
    if(no==SC_UNKNOWN_LEFT_FACE_REAR) :
        xd = p.L/4
        yd = -p.W/2
    
    xdd = xd*math.cos(t) - yd*math.sin(t)
    ydd = xd*math.sin(t) + yd*math.cos(t)
    return x + xdd, y + ydd

def under_ll_turn(x,y):
    if y<-2120 and x<-258 :
        return True
    else :
        return False