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

rvy1 = 0.0426
rvy2 = 0.03077
rvy3 = 0.37305
rvy4 = 100
rvy5 = 2.2
rvy6 = 25

ssz1 = -0.09916
ssz2 = 0.025876
ssz3 = 2.2703
ssz4 = -1.8657

qhz1 = 0.015524
qhz2 = -0.009173
qhz3 = -0.07129
qhz4 = 0.034226

pkx1 = 63.75
pkx2 = -15
pkx3 = 0.2891

qdz1 = 0.11496
qdz2 = 0.005418
qdz3 = 1.5
qdz4 = 0
qdz6 = -0.00023155
qdz7 = -0.02192
qdz8 = -1.3554
qdz9 = 0.119

qbz1 = 12.457
qbz2 = -0.04661
qbz3 = 0
qbz4 = 5.096
qbz5 = 4.664
qbz9 = 6.924
qbz10 = 0

qcz1 = 1.6

qez1 = -10
qez2 = 5.742
qez3 = 0
qez4 = 1.3084
qez5 = 1.2514






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

# def calc_alliging_torque(Fx, Fy, slip_angle, slip_ratio, fz, fz0, gamma, ):
#     epsilon = 0.000
    
#     // Aligning torque
#     gamma = ########

#     #variable req r0, ls
#     Cy = pcy1
#     muy = pdy1 + pdy2*dfz
#     Dy = muy*fz
#     K = fz0*pky1*sin(2*atan(fz/(pky2*fz0)))    #+pky2*dfz)#*exp(pky3*dfz)
#     By=K/(Cy*Dy+epsilon)
#     Dvyk = muy*fz*(rvy1+rvy2*dfz+rvy3*gamma)*cos(atan(rvy4*slip_angle))
#     Svyk = Dvyk*sin(rvy5*atan(rvy6*slip_ratio))
    
#     lr=1
#     Fzn=fz*1000
#     dfz = (fz-fz0)/fz0
#     Sht=qhz1+qhz2*dfz
#     svy = fz*(pvy1+pvy2*dfz)
#     shy = phy1 + phy2*dfz 
#     alpha_t=slip_angle+Sht;

#     Dr=Fzn*((qdz6+qdz7*dfz)*lr)*r0        #*cosf(alpha_t) for Pac2006 (in MF52 this is still in Mzr=... below)
#     Kx=Fzn*(pkx1+pkx2*dfz)*expf(pkx3*dfz)
#     Ky=pky1*fz0*sinf(2*atanf(Fzn/(pky2*fz0)))    // =BCD=stiffness at slipangle 0

#     Br=qbz9+qbz10*By*Cy;

#     Bt=(qbz1+qbz2*dfz+qbz3*dfz*dfz)    // Pac2006 adds gamma_y^2 dependence (qbz6)
#     Ct=qcz1
#     Dt=Fzn*(qdz1+qdz2*dfz)*(r0/fz0)*lt;        # lt=lamba_t?
#     Et=(qez1+qez2*dfz+qez3*dfz*dfz)*(1+(qez4)*(2/3.14159265)*atanf(Bt*Ct*alpha_t));   #<=1
#     # Clamp Et (eq 51)
#     if(Et>1):
#         Et=1
#     # Avoid division by zero
#     if(fabs(Ky)<0.001)
#     if(Ky<0)Ky=-0.001
#     else    Ky=0.001

#     Shf=shy+svy/Ky
#     alpha_r=slip_angle+Shf
    
#     if(alpha_t>=0)sign_alpha_t=1
#     else          sign_alpha_t=-1
#     if(alpha_r>=0)sign_alpha_r=1
#     else          sign_alpha_r=-1
#     kk=Kx/Ky
#     kk=(kk*kk*slip_ratio*slip_ratio)     // kk^2*slip_ratio^2
#     alpha_r_eq=sqrtf(alpha_r*alpha_r+kk)*sign_alpha_r
#     alpha_t_eq=sqrtf(alpha_t*alpha_t+kk)*sign_alpha_t
#     s=(ssz1+ssz2*(Fy/fz0)+(ssz3+ssz4*dfz)*gamma)*R0*ls
#     Mzr=Dr*cosf(atanf(Br*alpha_r_eq))*cosf(slip_angle)
#     Fy_der=Fy-Svyk
#     // New pneumatic trail
#     tmp=Bt*alpha_t_eq
#     t=Dt*cosf(Ct*atanf(tmp-Et*(tmp-atanf(tmp))))*cosf(slip_angle)

#     // Add all aligning forces
#     Mz=-t*Fy_der+Mzr+s*Fx

#    // Postprocess; negate for Racer
#    Fy=-Fy
#    Mz=-Mz

def sigmoid(x) :
    # return if_else(x<0,SX.exp(x)/(SX.exp(x)+1),1/(SX.exp(-x)+1)) 
    return SX.exp(x)/(SX.exp(x)+1) 

def inside_region_2(px,py):
    if py < -1144 and px < -240 and py > -1470:
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

def get_gyk(slip_angle,fz,fz0,slip_ratio):
    epsilon = 0.000
    dfz = (fz-fz0)/fz0
    shy = phy1 + phy2*dfz
    ky = slip_angle + shy
    Cy = pcy1
    muy = pdy1 + pdy2*dfz
    # print("muy :", muy)
    Dy = muy*fz
    Ey = (pey1 + pey2*dfz)*(1+pey3*(2*(ky<0)-1))
    K = fz0*pky1*sin(2*atan(fz/(pky2*fz0)))#+pky2*dfz)#*exp(pky3*dfz)
    By=K/(Cy*Dy+epsilon)
    svy = fz*(pvy1+pvy2*dfz)
    fy0 = (Dy*sin(Cy*atan(By*ky- Ey*(By*ky-atan(By*ky)))) + svy)
    # return Fy

    # const req rvy1, rvy2, rvy3,  rvy4,  rvy5, rvy6, lvyka, lyka
    #variable req gamma(maybe 0), alpha(slip_angle), k(slip_ratio),
    gamma = 0
    Dvyk = muy*fz*(rvy1+rvy2*dfz+rvy3*gamma)*cos(atan(rvy4*slip_angle))
    Svyk = Dvyk*sin(rvy5*atan(rvy6*slip_ratio))
    Shyk = rhy1+rhy2*dfz
    Eyk = rey1+rey2*dfz
    Cyk = rcy1
    Byk = rby1*cos(atan(rby2*(slip_angle-rby3)))
    ks = slip_ratio+Shyk
    Gyk0 = cos(Cyk*atan(Byk*Shyk-Eyk*(Byk*Shyk-atan(Byk*Shyk))))
    Gyk = cos(Cyk*atan(Byk*ks-Eyk*(Byk*ks-atan(Byk*ks))))/Gyk0
    # print(Gyk,Svyk)
    Fy = Gyk*fy0+Svyk
    return Gyk

def calc_force_from_slip_ratio(slip_angle,fz,fz0,slip_ratio):
    epsilon = 0.000
    dfz = (fz-fz0)/fz0
    shy = phy1 + phy2*dfz
    ky = slip_angle + shy
    Cy = pcy1
    muy = pdy1 + pdy2*dfz
    # print("muy :", muy)
    Dy = muy*fz
    Ey = (pey1 + pey2*dfz)*(1+pey3*(2*(ky<0)-1))
    K = fz0*pky1*sin(2*atan(fz/(pky2*fz0)))#+pky2*dfz)#*exp(pky3*dfz)
    By=K/(Cy*Dy+epsilon)
    svy = fz*(pvy1+pvy2*dfz)
    fy0 = (Dy*sin(Cy*atan(By*ky- Ey*(By*ky-atan(By*ky)))) + svy)
    # return Fy

    # const req rvy1, rvy2, rvy3,  rvy4,  rvy5, rvy6, lvyka, lyka
    #variable req gamma(maybe 0), alpha(slip_angle), k(slip_ratio),
    gamma = 0
    Dvyk = muy*fz*(rvy1+rvy2*dfz+rvy3*gamma)*cos(atan(rvy4*slip_angle))
    Svyk = Dvyk*sin(rvy5*atan(rvy6*slip_ratio))
    Shyk = rhy1+rhy2*dfz
    Eyk = rey1+rey2*dfz
    Cyk = rcy1
    Byk = rby1*cos(atan(rby2*(slip_angle-rby3)))
    ks = slip_ratio+Shyk
    Gyk0 = cos(Cyk*atan(Byk*Shyk-Eyk*(Byk*Shyk-atan(Byk*Shyk))))
    Gyk = cos(Cyk*atan(Byk*ks-Eyk*(Byk*ks-atan(Byk*ks))))/Gyk0
    # print(Gyk,Svyk)
    Fy = Gyk*fy0+Svyk
    return p.road_coeff*Fy

def calc_force_from_slip_angle(slip_angle,fz,fz0,curr_road_coeff):
    epsilon = 0.000
    dfz = (fz-fz0)/fz0
    # dfz=0
    shy = phy1 + phy2*dfz
    ky = slip_angle + shy
    Cy = pcy1
    muy = pdy1 + pdy2*dfz
    # print("muy :", muy)
    Dy = muy*fz
    Ey = (pey1 + pey2*dfz)*(1+pey3)
    K = fz0*pky1*sin(2*atan(fz/(pky2*fz0)))#+pky2*dfz)#*exp(pky3*dfz)
    By=K/(Cy*Dy+epsilon)
    svy = fz*(pvy1+pvy2*dfz)
    # print(slip_angle,fz,dfz,By*ky)
    Fy = Dy*sin(Cy*atan(By*ky - Ey*(By*ky-atan(By*ky)))) + svy
    return curr_road_coeff*Fy

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
                [-17.1,1.9,11.4,17.1,19,26.6,34.2,38,43.7,51.3,52.25],            
                [-15.2,3.8,15.2,19,20.9,28.5,36.1,39.9,44.65,52.25,53.01],
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