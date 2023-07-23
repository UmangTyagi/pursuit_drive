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
        
        self.Kp = 1
        self.Ki = 0.1
        self.Kd = 0.01
        self.dt = dt
        self.e_k_1 = e_k_1
        self.E_k_1 = E_k_1
        
        self.C_rr = 1
        self.C_aero = 1
        self.rho = 1
        self.A = 1
        self.r_eff = 1
        self.GR = 1
    
    def wrap_to_pi(self):
        
        if self.rotation.yaw > np.pi:
            self.rotation.yaw -= 2*np.pi
            
        elif self.rotation.yaw < -np.pi:
            self.rotation.yaw += 2*np.pi
        
        else:
            pass
        
        return self.rotation
    
    def calculate_rr_axle_pos(self):
        
        self.rr_axle_pose.x = self.pose.x - self.wheelbase/2 * np.cos(self.rotation.yaw)
        self.rr_axle_pose.y = self.pose.y + self.wheelbase/2 * np.sin(self.rotation.yaw)
    
        return self.rr_axle_pose
    
    def calculate_lookahead_distance(self):
        # takes the next waypoint and rear axle pos as input
        # returns lookahead distance (distance bw host's rear axle pos and the next waypoint)
        self.lookahead_dist = np.sqrt((self.rr_axle_pose.x - self.next_waypoint.x) + (self.rr_axle_pose.y - self.next_waypoint.y))
        return self.lookahead_dist
    
    def calculate_alpha(self, wheelbase, lookahead_dist):
        # takes the host's wheelbase and current lookahead distance as input
        # returns alpha
        self.alpha = np.arccos(wheelbase / lookahead_dist)
        return self.alpha
    
    def calculate_req_str_angle(self):
        # takes host's wheelbase, alpha (angle bw host's direction and waypoint's direction, host's long vel)
        # returns requored steering angle in radians
        self.req_str_angle_rad = np.atan2(2*self.wheelbase*np.sin(self.alpha), self.kpp * self.long_vel)
        return self.req_str_angle_rad
    
    def lateral_controller(self):
        # calculates the lookahead distance, alpha and req steering angle for a single cycle
        self.rotation = self.wrap_to_pi(self)
        
        self.rr_axle_pose = self.calculate_rr_axle_pos(self)
        
        self.lookahead_dist = self.calculate_lookahead_distance(self.next_waypoint, self.rr_axle_pose)
        
        self.alpha = self.calculate_alpha(self.wheelbase, self.lookahead_dist)
        
        self.req_str_angle_rad = self.calculate_req_str_angle(self.wheelbase, self.alpha, self.long_vel)
        
        return self.req_str_angle_rad
    
    
    def PID_controller(self):
        
        self.e_k = self.long_vel_ref - self.long_vel # velocity error
        self.E_k = self.E_k_1 + self.e_k * self.dt # cumulative error
        self.e_k_dot = (self.e_k - self.e_k_1)/self.dt # derivative error
        
        self.req_acc =self.Kp *self.e_k + self.Ki * self.E_k + self.Kd * self.e_k_dot # required acceleration
        
        return self.req_acc
    
    def calculate_long_vel_ref(self):
        self.long_vel_ref = 2 ## add here
        
        return self.long_vel_ref
    
    def slave_controller(self):
        
        self.Ft = self.mass_of_vehicle * self.req_acc + self.C_rr * self.mass_of_vehicle * self.g + 0.5*self.rho*self.A*self.long_vel_ref**2 * self.C_aero
        
        self.T_motor = self.Ft * self.r_eff * self.GR
        
        self.throttle = np.tanh(self.T_motor)
        
        return self.throttle
    
    
        
        