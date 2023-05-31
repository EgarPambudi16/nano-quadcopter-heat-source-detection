import numpy as np
import cv2
import logging
import time
import random
import sys
from threading import Event
from simple_pid import PID

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper

import vision_logging as vl

## Initialize Crazyflie
###########################################################
URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
deck_attached_event = Event()
logging.basicConfig(level=logging.ERROR)

## Initialize Variable
###########################################################
DEFAULT_HEIGHT = 0.8
lat_vel = 0.1
take_off_status = False
heat_status = False
motion_status = 0
position_estimate = [0, 0, 0]
heading_angle = 0
heading_limit = 0
x_dis_arr = np.zeros(10)
y_dis_arr = np.zeros(10)
xi = 0
yi = 0
pixels = np.zeros(64)
norm_pix = []
temp_threshold = 30
time_prev = 0
time_now = 0
time_total = 0
time_elapsed = 0
log_file = "Code/Log/log15.csv"

## Logging Function
###########################################################
def log_pos_callback(timestamp, data, logconf):
    # print(data)
    global position_estimate
    position_estimate[0] = data['stateEstimate.x']
    position_estimate[1] = data['stateEstimate.y']
    position_estimate[2] = data['stateEstimate.z']
    log_file_handler.write("{}, {}, {}, {}\n".format(position_estimate[0], position_estimate[1], position_estimate[2], motion_status))

def log_amg_callback1(timestamp, data, logconf):
    #print(data)
    global pixels
    for x in range (0, 16):
        pixels[63-x] = data['amgdeck.new_pixels[' + str(x) + ']']

def log_amg_callback2(timestamp, data, logconf):
    #print(data)
    global pixels
    for x in range (16, 32):
        pixels[63-x] = data['amgdeck.new_pixels[' + str(x) + ']']

def log_amg_callback3(timestamp, data, logconf):
    #print(data)
    global pixels
    for x in range (32, 48):
        pixels[63-x] = data['amgdeck.new_pixels[' + str(x) + ']']

def log_amg_callback4(timestamp, data, logconf):
    #print(data)
    global pixels
    for x in range (48, 64):
        pixels[63-x] = data['amgdeck.new_pixels[' + str(x) + ']']

def param_deck_flow(_, value_str):
    value = int(value_str)
    print(value)
    if value:
        deck_attached_event.set()
        print('Deck is attached!')
    else:
        print('Deck is NOT attached!')

def param_deck_amg(_, value_str):
    value = int(value_str)
    print(value)
    if value:
        deck_attached_event.set()
        print('AMG is attached!')
    else:
        print('AMG is NOT attached!')

def logconf_amg_init(period):
    logconf_amg1 = LogConfig(name='Pixels', period_in_ms=period)
    for x in range (0, 16):
        amg_variable = 'amgdeck.new_pixels[' + str(x) + ']'
        logconf_amg1.add_variable(amg_variable, 'uint8_t')
    scf.cf.log.add_config(logconf_amg1)
    logconf_amg1.data_received_cb.add_callback(log_amg_callback1)

    logconf_amg2 = LogConfig(name='Pixels', period_in_ms=period)
    for x in range (16,32):
        amg_variable = 'amgdeck.new_pixels[' + str(x) + ']'
        logconf_amg2.add_variable(amg_variable, 'uint8_t')
    scf.cf.log.add_config(logconf_amg2)
    logconf_amg2.data_received_cb.add_callback(log_amg_callback2)

    logconf_amg3 = LogConfig(name='Pixels', period_in_ms=period)
    for x in range (32,48):
        amg_variable = 'amgdeck.new_pixels[' + str(x) + ']'
        logconf_amg3.add_variable(amg_variable, 'uint8_t')
    scf.cf.log.add_config(logconf_amg3)
    logconf_amg3.data_received_cb.add_callback(log_amg_callback3)

    logconf_amg4 = LogConfig(name='Pixels', period_in_ms=period)
    for x in range (48,64):
        amg_variable = 'amgdeck.new_pixels[' + str(x) + ']'
        logconf_amg4.add_variable(amg_variable, 'uint8_t')
    scf.cf.log.add_config(logconf_amg4)
    logconf_amg4.data_received_cb.add_callback(log_amg_callback4)

    return logconf_amg1, logconf_amg2, logconf_amg3, logconf_amg4    

def logconf_pos_init(period):
    logconf_pos = LogConfig(name='Position', period_in_ms=20)
    logconf_pos.add_variable('stateEstimate.x', 'float')
    logconf_pos.add_variable('stateEstimate.y', 'float')
    logconf_pos.add_variable('stateEstimate.z', 'float')
    scf.cf.log.add_config(logconf_pos)
    logconf_pos.data_received_cb.add_callback(log_pos_callback)
    return logconf_pos

## Control and Motion Function
###########################################################
def pid_controller(x_distance, y_distance):
    global xi, yi
    xi = (xi+1) % 10
    yi = (yi+1) % 10
    x_dis_arr[xi] = x_distance
    y_dis_arr[yi] = y_distance
    x_ave = np.average(x_dis_arr)
    y_ave = np.average(y_dis_arr)

    pid_z_vel = PID(Kp=0.0003, Ki=0, Kd=0.0001, setpoint=0, sample_time=0.05)
    pid_yaw_rate = PID(Kp=0.1, Ki=0, Kd=0, setpoint=0, sample_time=0.05)
    z_vel = pid_z_vel(-y_ave)
    yaw_rate = pid_yaw_rate(-x_ave)

    return z_vel, yaw_rate

def motion_run(mc):
    global motion_status
    motion_status = 1
    mc.start_linear_motion(velocity_x_m=lat_vel, velocity_y_m=0, velocity_z_m=0, rate_yaw=0)
    # print("Crazyflie Running!")

def motion_tumble(mc):
    global motion_status
    global heading_angle
    print(heading_angle)
    motion_status = 2
    r = random.random()
    if heading_angle == 0:
        if r > 0.5:
            heading_angle = 90*r
            mc.turn_right(90*r, rate=45)
        else:
            heading_angle = (-90*r)
            mc.turn_left(90*r, rate=45)
    elif heading_angle > 90 and heading_angle > 0:
        mc.turn_left(180, rate=60)
        mc.forward(0.3)
        heading_angle -= 180
        print(1)
    elif heading_angle < -90 and heading_angle < 0:
        mc.turn_right(180, rate=60)
        heading_angle += 180
        mc.forward(0.3)
        print(2)
    elif heading_angle < 0:
        if r > 0.3:
            heading_angle += 120*r
            mc.turn_right(120*r, rate=45)
            print(3)
        else:
            heading_angle -= 45*r
            mc.turn_left(45*r, rate=45)
            print(4)
    elif heading_angle > 0:
        if r > 0.3:
            heading_angle -= 120*r
            mc.turn_left(120*r, rate=45)
            print(5)
        else:
            heading_angle += 45*r
            mc.turn_right(45*r, rate=45)
            print(6)

    mc.stop()
    time.sleep(0.5) 

def motion_centering(mc, z_vel, yaw_rate):
    global motion_status
    motion_status = 3
    # print("Crazyflie Centering!")
    mc.start_linear_motion(0, 0, z_vel, yaw_rate)

def motion_chase(mc, z_vel, yaw_rate):
    global motion_status
    motion_status = 4
    mc.start_linear_motion(velocity_x_m=lat_vel, velocity_y_m=0, velocity_z_m=z_vel, rate_yaw=yaw_rate)
    # print("Crazyflie Chasing!")

if __name__ == '__main__':

    ## Initialize Crazyflie Connection and Parameter
    ###########################################################
    cflib.crtp.init_drivers()
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        scf.cf.param.add_update_callback(group='deck', name='bcFlow2',
                                         cb=param_deck_flow) 
        scf.cf.param.add_update_callback(group='deck', name='amgdeck',
                                         cb=param_deck_amg)
        time.sleep(1)

        logconf_pos = logconf_pos_init(20)
        logconf_amg1, logconf_amg2, logconf_amg3, logconf_amg4 = logconf_amg_init(100)

        if not deck_attached_event.wait(timeout=5):
            print('No flow deck detected!')
            sys.exit(1)

        log_file_handler = open(log_file, "w")
        log_file_handler.close()
        log_file_handler = open(log_file, "a")

        logconf_pos.start()
        logconf_amg1.start()
        logconf_amg2.start()
        logconf_amg3.start()
        logconf_amg4.start()
        
        ## Fly and Detect Sequence
        ###########################################################
        with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
            while True:
                frame, thres_bgr, contours, temp = vl.amg_process(pixels, temp_threshold)
                length = len(contours)
                maxArea = -1
                if length > 0:
                    heat_status = True
                    for i in range(length):  # Finding the Biggest Contour
                        con_temp = contours[i]
                        area = cv2.contourArea(con_temp)
                        if area > maxArea:
                            maxArea = area
                            ci = i

                    res = contours[ci]
                    (x, y, w, h) = cv2.boundingRect(res)
                    x_center = int(x+w/2)
                    y_center = int(y+h/2)
                    x_amg_distance = int(x_center - 400)
                    y_amg_distance = int(400 - y_center)
                    z_vel, yaw_rate = pid_controller(x_amg_distance, y_amg_distance)
                    # motion_centering(mc, z_vel, yaw_rate)

                    if maxArea > 250000 or temp > 65:
                        for i in range (0, 6):
                            mc.stop()
                            time.sleep(0.5)
                            if maxArea < 100000 or temp < 50:
                                break
                        break
                    elif x_amg_distance > 40 or y_amg_distance > 40:
                        motion_centering(mc, z_vel, yaw_rate)
                    else:
                        motion_chase(mc, z_vel, yaw_rate)

                    thres_bgr = vl.threshold_box(thres_bgr, res, (x, y, w, h), (x_center, y_center))

                else:
                    heat_status = False
                    # motion_run(mc)
                    # motion_tumble(mc)
                    # mc.stop()
                    time_now = time.time()
                    time_total += time_now - time_prev
                    time_prev = time.time()
                    if time_total < 1:
                        motion_run(mc)
                    else:
                        time_total = 0
                        mc.stop()
                        motion_tumble(mc)
                
                # print(time_total)
                # # print(time_elapsed)
                # if time_elapsed > 0.05:
                #     time_elapsed = 0
                #     log_file_handler.write("{}, {}, {}, {}\n".format(position_estimate[0], position_estimate[1], position_estimate[2], motion_status))
                thres_bgr = vl.threshold_gui(thres_bgr, temp, temp_threshold, motion_status, position_estimate)
                cv2.imshow("Heat Detection Frame", thres_bgr)
                cv2.imshow("AMG8833 Datastream Frame", frame)
                time_prev = time.time()
                if cv2.waitKey(1) == 27:  
                    break

            print("Heatsource Relative Position:", position_estimate)
            motion_status = 5
            mc.back(0.2)
            mc.down(0.4, 0.4)
            mc.land(0.4)

        logconf_pos.stop()
        logconf_amg1.stop()
        logconf_amg2.stop()
        logconf_amg3.stop()
        logconf_amg4.stop()
        log_file_handler.close()
        sys.exit(1)
