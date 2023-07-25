import numpy as np



class controller_2d:
    '''
    class containing methods for calculating lateral and longitudinal control output for the host based on its 
    current position w.r.t to the world coordinate system in Carla and its current states - long speed

    '''
    
    
    def __init__(self,waypoints, long_vel, pose, rotation, dt, e_k_1, E_k_1 ):
        
        self.mass_of_vehicle = 1
        self.g = 9.81
        self.wheelbase =1
        self.pose = pose
        self.rotation = rotation
        self.long_vel = long_vel
        
        self.kpp = 1
        self.next_waypoint = waypoints[0]
        self.waypoints = waypoints
        self.max_lat_acc = 7 # m/s2
        
        self.Kp = 1
        self.Ki = 0.1
        self.Kd = 0.01
        self.dt = dt
        self.e_k_1 = e_k_1
        self.E_k_1 = E_k_1
        
        self.C_rr = 1
        self.C_aero = 1
        self.A = 1
        self.r_eff = 1
        self.GR = 1
        
        self.req_str_angle_rad = 0
        self.long_vel_ref = 0
        self.throttle = 0
        self.ROC = 0
        
        
    def wrap_to_pi(self, yaw):
        '''
        description : wraps the angle bw -pi to pi
        input : yaw
        output : wrapped to pi yaw
        '''
        if yaw > np.pi:
            yaw -= 2*np.pi
            
        elif yaw < -np.pi:
            yaw += 2*np.pi
        else:
            pass
        
        return yaw
    
    def calculate_rr_axle_pos(self, pose, wheelbase, yaw):
        '''
        description : calculates rear axle position
        input : host's current position (center), wheelbase and yaw
        output : rear axle position
        '''
        rr_axle_pose_x = pose.x - wheelbase/2 * np.cos(yaw)
        rr_axle_pose_y = pose.y - wheelbase/2 * np.sin(yaw)
    
        return rr_axle_pose_x, rr_axle_pose_y
    
    def calculate_lookahead_distance(self, rr_axle_pose_x, rr_axle_pose_y, next_waypoint):
        '''
        description : calculates lookahead distance
        input : rear axle position and next waypoint
        output : lookahead distance
        '''
        lookahead_dist = np.sqrt((rr_axle_pose_x - next_waypoint.x) + (rr_axle_pose_y - next_waypoint.y))
        
        return lookahead_dist
    
    def calculate_alpha(self, wheelbase, lookahead_dist):
        '''
        description : calculates alpha (angle bw the host's long axis and line joining rear axle and the waypoint')
        input : wheelbase and lookahead distance
        output : alpha
        '''
        alpha = np.arccos(wheelbase / lookahead_dist)
        
        return alpha
    
    def calculate_req_str_angle(self, wheelbase, alpha, kpp, long_vel):
        '''
        description : calculates the required steering using pure pursuit algorithm
        input : wheelbase, alpha, kpp (coefficient pure pursuit) and long vel (actual)
        output : 
        '''
        req_str_angle_rad = np.atan2(2*wheelbase*np.sin(alpha), kpp * long_vel)
        
        return req_str_angle_rad
    
    def lateral_controller(self, wrap_to_pi, calculate_rr_axle_pos, calculate_lookahead_distance, calculate_alpha, calculate_req_str_angle): 
        '''
        description : lateral controller (combined methods)
        input : lateral control methods
        output : required steering angle
        '''
        yaw = wrap_to_pi(self.rotation.yaw)
        
        rr_axle_pose_x, rr_axle_pose_y = calculate_rr_axle_pos(self.pose, self.wheelbase, yaw)
        
        lookahead_dist = calculate_lookahead_distance(rr_axle_pose_x, rr_axle_pose_y, self.next_waypoint)
        
        alpha = calculate_alpha(self.wheelbase, lookahead_dist)
        
        self.req_str_angle_rad = calculate_req_str_angle(self.wheelbase, alpha, self.kpp, self.long_vel)
        
        return self.req_str_angle_rad
    
    
    def PID_controller(self, long_vel_ref, long_vel, Kp, Ki, Kd, E_k_1, e_k_1, dt):
        '''
        description : calculates required acceleration using a PID controller
        input : reference long velocity, actual long velocity, PID coefficients and errors and dt
        output : required long acceleration
        '''
        e_k = long_vel_ref - long_vel # velocity error
        E_k = E_k_1 + e_k * dt # cumulative error
        e_k_dot = (e_k - e_k_1)/dt # derivative error
        
        req_acc =self.Kp *e_k + self.Ki * E_k + self.Kd * e_k_dot # required acceleration
        
        return req_acc
    
    def fit_circle(self, waypoints):
        '''
        description : fits the next 3 waypoints into a circle to find the radius of curvature
        input : next 3 waypoints
        output : radius of curvature
        '''
        x1 = waypoints[0][0]
        y1 = waypoints[0][1]
        y2 = waypoints[1][1]
        x3 = waypoints[2][0]
        y3 = waypoints[2][1]
        x2 = waypoints[1][0]
        
        A = x1*(y2-y3) - y1*(x2-x3) + x2*y3 - x3*y2
        B = (x1**2 + y1**2)*(y3-y2) + (x2**2 + y2**2)*(y1-y3) + (x3**2 + y3**2)*(y2-y1)
        C = (x1**2 + y1**2)*(x2-x3) + (x2**2 + y2**2)*(x3-x1) + (x3**2 + y3**2)*(x1-x2)
        D = (x1**2 + y1**2)*(x3*y2 - x2*y3) + (x2**2 + y2**2)*(x1*y3 - x3*y1) + (x3**2 + y3**2)*(x2*y1 - x1*y2)        
        
        # xc = -B/(2*A)
        # yc = -C/(2*A)
        self.ROC = np.sqrt((B**2 + C**2 - 4*A*D)/(4*A**2))
        
        return self.ROC
        
    def calculate_long_vel_ref(self, max_lat_acc, ROC):
        '''
        description : calculates reference long velocity
        input : max lateral acceleration and radisu of curvature
        output : reference long vel
        '''
        self.long_vel_ref = np.sqrt(max_lat_acc * ROC)
        
        return self.long_vel_ref
    
    def slave_controller(self, mass_of_vehicle, req_acc, C_rr, C_aero, A, long_vel_ref, r_eff, GR):
        '''
        description : slave controller of the longitudinal controller
        input : mass of vehicle, required long acceleration, rolling resistance and aero drag coefficients, ref long vel, efective tire radius and overall gear ratio
        output : throttle (0 to 1)
        '''
        g = 9.81 #m/s2
        rho = 1
        
        Ft = mass_of_vehicle * req_acc + C_rr * mass_of_vehicle * g + 0.5*rho*A*long_vel_ref**2 * C_aero
        
        T_motor = Ft * r_eff * GR
        
        self.throttle = np.tanh(T_motor)
        
        return self.throttle
    
    def reset(self):
        '''
        description : resets the attributes
        '''
        self.req_str_angle_rad = 0
        self.long_vel_ref = 0
        self.throttle = 0
        self.ROC = 0
                
