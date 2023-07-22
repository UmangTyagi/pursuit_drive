import numpy as np



class controller_2d:
    '''
    class containing methods for calculating lateral and longitudinal control output for the host based on its 
    current position w.r.t to the world coordinate system in Carla and its current states - long speed

    '''
    
    
    def __init__(self,x ):
        self.x = x
        self.kpp = 1
        
    def calculate_req_str_angle(self, wheelbase, alpha, long_vel):
        # takes host's wheelbase, alpha (angle bw host's direction and waypoint's direction, host's long vel)
        # returns requored steering angle in radians
        req_str_angle_rad = np.atan2(2*wheelbase*np.sin(alpha), self.kpp * long_vel)
        return req_str_angle_rad
    
    def calculate_lookahead_distance(self, next_waypoint, actor_rr_axle_pose):
        # takes the next waypoint and rear axle pos as input
        # returns lookahead distance (distance bw host's rear axle pos and the next waypoint)
        lookahead_dist = np.sqrt((actor_rr_axle_pose.x - next_waypoint.x) + (actor_rr_axle_pose.y - next_waypoint.y))
        return lookahead_dist
    
    def calculate_alpha(self, wheelbase, lookahead_dist):
        # takes the host's wheelbase and current lookahead distance as input
        # returns alpha
        alpha = np.arccos(wheelbase / lookahead_dist)
        return alpha
    
    def lateral_controller(self, wheelbase, next_waypoint, actor_rr_axle_pose, long_vel):
        # calculates the lookahead distance, alpha and req steering angle for a single cycle
        lookahead_dist = self.calculate_lookahead_distance(next_waypoint, actor_rr_axle_pose)
        
        alpha = self.calculate_alpha(wheelbase, lookahead_dist)
        
        req_str_angle_rad = self.calculate_req_str_angle(wheelbase, alpha, long_vel)
        
        return req_str_angle_rad
        
        