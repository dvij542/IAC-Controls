import yaml

# with open('params.yaml') as f:
f = open('params.yaml')
params = yaml.load(f, Loader=yaml.FullLoader)

manual_gear_change = params['parameters']['manual_gear_change']
gear_change_engine_thres= params['parameters']['gear_change_engine_thres']
start_throttle = params['parameters']['start_throttle']
start_speed = params['parameters']['start_speed']

road_coeff = params['parameters']['road_coeff']
vehicle_length_r = params['parameters']['vehicle_length_r']
blocking_maneuver_cost = params['parameters']['blocking_maneuver_cost']
air_resistance_const = params['parameters']['air_resistance_const']
mass = params['parameters']['mass']
tolerance = params['parameters']['tolerance']
Q_ang = params['parameters']['Q_ang']
k_lat_slip = params['parameters']['k_lat_slip']
k_vel_follow = params['parameters']['k_vel_follow']
save_path_after = params['parameters']['save_path_after']
file_path_follow = params['parameters']['file_path_follow']
file_new_path = params['parameters']['file_new_path']
Q_along = params['parameters']['Q_along']
Q_dist = params['parameters']['Q_dist']
penalty_out_of_road = params['parameters']['penalty_out_of_road']
no_iters = params['parameters']['no_iters']
max_no_of_vehicles = params['parameters']['max_no_of_vehicles']

T = params['parameters']['T']
N = params['parameters']['N']
kp = params['parameters']['kp']
obs_dist = params['parameters']['obs_dist']
ki = params['parameters']['ki']
kd = params['parameters']['kd']
threshold = params['parameters']['threshold']
dist_threshold = params['parameters']['dist_threshold']
kp_start = params['parameters']['kp_start']
ki_start = params['parameters']['ki_start']
kd_start = params['parameters']['kd_start']
I_start = params['parameters']['I_start']
lift_coeff = params['parameters']['lift_coeff']
lift_coeff_r = eval(params['parameters']['lift_coeff_r'])
lift_coeff_f = eval(params['parameters']['lift_coeff_f'])
gravity_constant = params['parameters']['gravity_constant']
pdy1 = params['parameters']['pdy1']
pdy2 = params['parameters']['pdy2']
fz0 = params['parameters']['fz0']
epsilon = params['parameters']['epsilon']
L = params['parameters']['L']

# Gear Params:
gear_throttles = params['Gear Params']['gear_throttles']
gear_change_speeds = params['Gear Params']['gear_change_speeds']
gear_radiis = params['Gear Params']['gear_radiis']
gear_speed_l = params['Gear Params']['gear_speed_l']
gear_speed_r = params['Gear Params']['gear_speed_r']