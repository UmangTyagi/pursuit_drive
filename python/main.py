import numpy as np
import os
import sys
import glob
import time
from controller_2d import controller_2d
try:
    sys.path.append(glob.glob('../../../Carla/CARLA_0.9.8/WindowsNoEditor/PythonAPI/carla/dist//carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

def setup():

    f_setup = False
    global actor, spectator, waypoints
    try:
        client = carla.Client('localhost', 2000)  #connecting to carla server
        client.set_timeout(10.0)
        print('Update: connected to carla server')

        try:
            world = client.load_world('Town07') #loading town 7
            map = world.get_map()
            print('Update: loaded world')
            # time.sleep(5)

            if world and map:
                blueprint_library = world.get_blueprint_library()
                vehicle_bp = blueprint_library.find('vehicle.tesla.model3')

                waypoints = map.generate_waypoints(distance = 5)

                spawn_point = waypoints[0].transform.location
                initial_yaw = waypoints[0].transform.rotation.yaw

                actor_transform = carla.Transform(carla.Location(x=spawn_point.x, y=spawn_point.y, z=1), carla.Rotation(pitch=0, yaw=initial_yaw, roll=0))

                try:
                    actor = world.spawn_actor(vehicle_bp, actor_transform)
                    print('Update: actor spawned')
                    if actor is not None:
                        actor_location = actor.get_location()
                        actor_rotation = actor.get_transform().rotation
                        time.sleep(5)

                        try:
                            spectator = world.get_spectator()
                            spectator_spawn_point = actor_location
                            spectator_transform = carla.Transform(carla.Location(x=spectator_spawn_point.x, y = spectator_spawn_point.y, z=40),carla.Rotation(pitch=-90))
                            spectator.set_transform(spectator_transform)
                            print('Update: spectator set')
                            time.sleep(5)
                            f_setup = True

                        except:
                            print('Error: spawning spectator')
                     
                except:
                    print('Error: spawning actor')   

    
        except:
            print('Error: loading world')

    except:
        print('Error: connecting to carla server')

    return f_setup

def move_spectator(last_pos):

    current_pos = actor.get_transform().location
    
    if last_pos is None:
        last_pos = current_pos

    if ((abs(last_pos.x - current_pos.x) > 0) and (abs(last_pos.y - current_pos.y) > 0)):

        new_spectator_transform = carla.Transform(carla.Location(x=current_pos.x, y=current_pos.y, z = 40), carla.Rotation(pitch=-90))
        spectator.set_transform(new_spectator_transform)
        time.sleep(0.5)
    
    last_pos = current_pos

def get_actor_states():

    current_pose = actor.get_transform().location
    current_rotation = actor.get_transform().rotation
    current_long_vel = actor.get_velocity().x

    return current_pose,current_rotation,current_long_vel

def get_waypoints(index):

    upcoming_waypoints = waypoints[index:index+3]

    return upcoming_waypoints

def main():

    f_setup = setup()
    # time.sleep(5)
    long_vel = 0
    pose = actor.get_transform().location
    rotation = actor.get_transform().rotation
    dt = 0.05
    e_k_1 = 0
    E_k_1 = 0

    next_3waypoint = waypoints[:3]
    controller = controller_2d(next_3waypoint, long_vel, pose, rotation, dt, E_k_1, e_k_1)

    control = carla.VehicleControl()
    index=1
    while index >=1:

        pose, rotation, long_vel = get_actor_states()
        
        E_k_1 = controller.E_k_1
        e_k_1 = controller.e_k_1

        next_3waypoint = get_waypoints(index)
        dt = 0.05

        controller.update(next_3waypoint, long_vel, pose, rotation, dt, E_k_1, e_k_1)

        str_angle = controller.lateral_controller()
        throttle = controller.longitudinal_controller()
        
        control.throttle = throttle
        control.steer = str_angle
        control.hand_brake = False
        control.reverse = False

        actor.apply_control(control)

        index += 1
        move_spectator(pose)
        print(f'scan : {index}    str')
    

if '__name__' == main():
    main()
