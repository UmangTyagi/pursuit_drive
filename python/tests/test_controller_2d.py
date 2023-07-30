from controller_2d import controller_2d
from dataclasses import dataclass
import pytest
import numpy as np

@dataclass
class mock_Pose:
    x: float = 0.0
    y: float = 0.0

@dataclass
class mock_Rotation:
    yaw: float = 0.0
    pitch: float = 0.0
    
@dataclass
class waypoint:
    x: float = 0.0
    y: float = 0.0

class Test_Controller_2d:
    '''
    test class for testing the methods of controller_2d class as seperate units
    function - setup_method initializes the inputs for the class controller_2d
    controller is an object/insatnce of controller_2d class which is used to call its methods
    '''
    def setup_method(self):
        self.waypoints = [[],[],[]]
        self.long_vel = 0
        self.pose = mock_Pose()
        self.rotation = mock_Rotation()
        self.dt = 0
        self.e_k_1 = 0
        self.E_k_1 = 0
        self.controller = controller_2d(self.waypoints, self.long_vel, self.pose, self.rotation, self.dt, self.e_k_1, self.E_k_1)
        
        
    '''
    test cases for method wrap_to_pi
    '''
    def test_wrap_to_pi_1(self):
        
        result = self.controller.wrap_to_pi(np.pi/2)
        expected_result = np.pi/2
        
        assert result == pytest.approx(expected_result)
        
    def test_wrap_to_pi_2(self):
        
        result = self.controller.wrap_to_pi(np.pi+1)
        expected_result = (np.pi+1) - 2*np.pi
        
        assert result == pytest.approx(expected_result, result)
        
    def test_wrap_to_pi_3(self):
        
        result = self.controller.wrap_to_pi(-(np.pi+2))
        expected_result = -(np.pi+2) + 2*np.pi
        
        assert result == pytest.approx(expected_result)
        
    '''
    test cases for calculate_rr_axle_pos
    '''
    def test_calculate_rr_axle_pos_1(self):

        self.controller.pose.x = 10
        self.controller.pose.y = 10
        self.controller.wheelbase = 4
        self.controller.rotation.yaw = np.deg2rad(45)
        
        expected_rr_axle_x = 10 - 2*np.cos(np.deg2rad(45))
        expected_rr_axle_y = 10 - 2*np.sin(np.deg2rad(45))
        
        rr_axle_x, rr_axle_y = self.controller.calculate_rr_axle_pos(self.controller.pose, self.controller.wheelbase, self.controller.rotation.yaw)
        
        assert rr_axle_x == pytest.approx(expected_rr_axle_x)
        assert rr_axle_y == pytest.approx(expected_rr_axle_y)

    def test_calculate_rr_axle_pos_2(self):

        self.controller.pose.x = 10
        self.controller.pose.y = 10
        self.controller.wheelbase = 4
        self.controller.rotation.yaw = np.deg2rad(90)
        
        expected_rr_axle_x = 10
        expected_rr_axle_y = 8
        
        rr_axle_x, rr_axle_y = self.controller.calculate_rr_axle_pos(self.controller.pose, self.controller.wheelbase, self.controller.rotation.yaw)
        
        assert rr_axle_x == pytest.approx(expected_rr_axle_x)
        assert rr_axle_y == pytest.approx(expected_rr_axle_y)

    '''
    test cases for calculate_lookahead_distance
    '''
    def test_calculate_lookahead_distance_1(self):
        
        rr_axle_pos_x = 10
        rr_axle_pos_y = 10
        self.next_waypoint = waypoint()
        self.next_waypoint.x = 20
        self.next_waypoint.y = 20

        expected_lookahead_distance = np.sqrt(200)

        lookahead_distance = self.controller.calculate_lookahead_distance(rr_axle_pos_x,rr_axle_pos_y,self.next_waypoint)

        assert lookahead_distance == pytest.approx(expected_lookahead_distance)

    def test_calculate_lookahead_distance_2(self):
        
        rr_axle_pos_x = 10
        rr_axle_pos_y = 10
        self.next_waypoint = waypoint()
        self.next_waypoint.x = 18
        self.next_waypoint.y = -25

        expected_lookahead_distance = np.sqrt(64 + 35**2)

        lookahead_distance = self.controller.calculate_lookahead_distance(rr_axle_pos_x,rr_axle_pos_y,self.next_waypoint)

        assert lookahead_distance == pytest.approx(expected_lookahead_distance)
