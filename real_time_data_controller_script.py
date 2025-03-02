#!/usr/bin/env python

#Importing libraries
import math
import sys
import logging
import matplotlib.pyplot as plt
import scipy
import numpy as np
from scipy import signal
import rtde.rtde as rtde
import rtde.rtde_config as rtde_config
from scipy.stats import norm

#Adding the path to the rtde library    
sys.path.append('..')
# logging.basicConfig(level=logging.INFO)

ROBOT_HOST = '192.168.5.1' #IP address of the robot 
ROBOT_PORT = 30004 #Port number of the robot
config_filename = 'control_loop_configuration-filtered.xml' #Configuration file for the control loop
keep_running = True #Boolean variable to keep the control loop running

limit = 1000 #Number of samples to collect

logging.getLogger().setLevel(logging.INFO) #Setting the logging level to INFO   
conf = rtde_config.ConfigFile(config_filename) #Loading the configuration file
state_names, state_types = conf.get_recipe('state') #Getting the state recipe
#setp_names, setp_types = conf.get_recipe('setp') #Getting the setpoint recipe

watchdog_names, watchdog_types = conf.get_recipe('watchdog') #Getting the watchdog recipe
con = rtde.RTDE(ROBOT_HOST, ROBOT_PORT) #Connecting to the robot    
con.connect() #Connecting to the robot  
con.get_controller_version() #Getting the controller version
# setup recipes
con.send_output_setup(state_names, state_types) #Sending the output setup
# setp = con.send_input_setup(setp_names, setp_types)
watchdog = con.send_input_setup(watchdog_names, watchdog_types) #Sending the watchdog setup
# The function "rtde_set_watchdog" in the "rtde_control_loop.urp" creates a 1 Hz watchdog

watchdog.input_double_register_23 = 0 #Setting the watchdog input to 0  
watchdog.input_double_register_24 = 0
watchdog.input_double_register_25 = 0
watchdog.input_double_register_26 = 0
watchdog.input_double_register_27 = 0
watchdog.input_double_register_28 = 0

# start data synchronization
if not con.send_start():
sys.exit()

x = np.array(range(limit)) #Creating an array of the limit

force_tcp = np.array([]) #Creating an array for the force in the TCP frame
force_actual_tcp = np.array([]) #Creating an array for the actual force in the TCP frame    
pose_actual_tcp = np.array([]) #Creating an array for the actual pose in the TCP frame  
speed_actual_tcp = np.array([]) #Creating an array for the actual speed in the TCP frame
pose_actual_joint = np.array([]) #Creating an array for the actual joint pose
velocity_actual_joint = np.array([]) #Creating an array for the actual joint velocity   
current_joint = np.array([]) #Creating an array for the current in the joint            
control_current_joint = np.array([]) #Creating an array for the control current in the joint
force_actual_z_tcp = np.array([]) #Creating an array for the actual force in the z direction in the TCP frame
tcp_z_force_25 = np.array([]) #Creating an array for the TCP z force            
force_torque_tool = np.array([]) #Creating an array for the force torque tool
var_z_log = np.array([]) #Creating an array for the variable z
error_F_log = [] #Creating an array for the error F
error_F_log.append(0) #Appending 0 to the error F log

var_kp_log = [] #Creating an array for the proportional gain
var_kp_log = np.array([]) #Creating an array for the proportional gain
var_ki_log = [] #Creating an array for the integral gain
var_ki_log.append(0) #Appending 0 to the integral gain log
var_kd_log = [] #Creating an array for the derivative gain
var_kd_log.append(0) #Appending 0 to the derivative gain log

#print(var_ki_log[-1])

####For Filtering

float_filtered_rows = [] #Creating an array for the filtered rows   
item_1D_log=np.array([]) #Creating an array for the item 1D log
sos_log=np.array([])

####For Mean
mean_log = [] #Creating an array for the mean log
#For loop to receive the current state
for i in range(limit):
##receive the current state
state = con.receive()

#####TCP force in TCP frame##################################################################

watch_out_25 = np.array([]) #Creating an array for the watch out 25
watch_out_25 = np.append(watch_out_25, state.output_double_register_25) #Appending the watch out 25 to the array
#print('watchout25',watch_out_25[-1])

####Filters######################################################################### #########

item_1D=[] #Creating an array for the item 1D
item_1D.append(state.output_double_register_25) #state.actual_TCP_force[2]
item_1D_log = np.append(item_1D_log,state.output_double_register_25)# state.actual_TCP_force

###Bessel filter
n_filter = 2 #Number of filter
wn_filter = 1.732 #Cutoff frequency 
sos = signal.bessel(n_filter, wn_filter, 'low',fs=50 , output='sos')#,norm='mag') #analog='true'
filtered_item_log = signal.sosfilt(sos, item_1D_log) #Filtering the item 1D log
x_log = np.array(range(1, len(item_1D_log) + 1)) #Creating an array for the x log   
print('Ff', filtered_item_log[-1]) #Printing the filtered item log
watchdog.input_double_register_25 = filtered_item_log[-1] #Setting the watchdog input to the filtered item log
float_filtered_rows.append(filtered_item_log[-1]) #Appending the filtered item log to the float filtered rows

####Controller##################################################################### #############

F_ref=35 #35
var_kp= 0.0050/F_ref #Proportional gain
var_ki= 0 #Integral gain
var_kd= 0 #Derivative gain
T_s= 1/20 #rtde 20Hz
print("Kp", var_kp) #Printing the proportional gain
error_F=abs(abs(filtered_item_log[-1])-F_ref) #Calculating the error F
var_z = ((var_kp * error_F)) * 0.001
#var_z = ((var_kp * error_F)+((var_kd/T_s)*(error_F-error_F_log[-1]))) * 0.001
#var_z=((var_kp*error_F)+((var_kd/T_s)*(error_F-error_F_log[-1]))+((var_ki*T_s/2)*(error_F2*error_F_log[-1]+error_F_log[-2]))+var_ki_log[-1])*0.001
print("var z",var_z)

var_kp_log = np.append(var_kp_log, var_kp) #Appending the proportional gain to the var kp log
var_ki_log.append(var_ki) #Appending the integral gain to the var ki log
var_kd_log.append(var_kd) #Appending the derivative gain to the var kd log
var_z_log = np.append(var_z_log, var_z) #Appending the variable z to the var z log
error_F_log.append(error_F) #Appending the error F to the error F log
watchdog.input_double_register_28 = var_z #Setting the watchdog input to the variable z

###Send watchdog to robot#############################################################

con.send(watchdog) #Sending the watchdog to the robot
if state is None: 
break;

###Saving the data to the csv files#############################################################
force_tcp = np.append(force_tcp, state.output_double_register_25) #Appending the force in the TCP frame to the force tcp
force_actual_tcp = np.append(force_actual_tcp, state.actual_TCP_force) #Appending the actual force in the TCP frame to the force actual tcp
pose_actual_tcp = np.append(pose_actual_tcp, state.actual_TCP_pose) #Appending the actual pose in the TCP frame to the pose actual tcp
speed_actual_tcp = np.append(speed_actual_tcp, state.actual_TCP_speed) #Appending the actual speed in the TCP frame to the speed actual tcp
pose_actual_joint = np.append(pose_actual_joint, state.actual_q) #Appending the actual joint pose to the pose actual joint
velocity_actual_joint = np.append(velocity_actual_joint, state.actual_qd)
current_joint = np.append(current_joint, state.actual_current) #Appending the actual current to the current joint
control_current_joint = np.append(control_current_joint, state.joint_control_output) #Appending the control current to the control current joint
force_actual_z_tcp = np.append(force_actual_z_tcp, state.actual_TCP_force[2]) #Appending the actual force in the z direction in the TCP frame to the force actual z tcp
np.savetxt('force_tcp_tool.csv', force_tcp, delimiter=',') #Saving the force tcp to the force tcp tool csv
np.savetxt('force_actual_tcp.csv', force_actual_tcp)
np.savetxt('pose_actual_tcp.csv', pose_actual_tcp, delimiter=',')
np.savetxt('speed_actual_tcp.csv', speed_actual_tcp, delimiter=',')
np.savetxt('pose_actual_joint.csv', pose_actual_joint, delimiter=',')
np.savetxt('velocity_actual_joint.csv', velocity_actual_joint, delimiter=',')
np.savetxt('current_joint.csv', current_joint, delimiter=',')
np.savetxt('control_current_joint.csv', control_current_joint, delimiter=',')
np.savetxt('force_actual_z_tcp.csv', force_actual_z_tcp, delimiter=',')
np.savetxt('float_filtered_rows.csv', float_filtered_rows, delimiter=',')
np.savetxt('var_z_log.csv', var_z_log, delimiter=',')
np.savetxt('var_kp_log.csv', var_kp_log, delimiter=',')
np.savetxt('var_kd_log.csv', var_kd_log, delimiter=',')
np.savetxt('var_ki_log.csv', var_ki_log, delimiter=',')
np.savetxt('error_F_log.csv', error_F_log, delimiter=',')

con.send_pause() #Sending the pause to the robot
con.disconnect()





'''
XML Code
<?xml version="1.0"?>
<rtde_config>
<recipe key="state">
<field name="output_double_register_23" type="DOUBLE"/>
<field name="output_double_register_24" type="DOUBLE"/>
<field name="output_double_register_25" type="DOUBLE"/>
<field name="output_double_register_26" type="DOUBLE"/>
<field name="output_double_register_27" type="DOUBLE"/>
<field name="output_double_register_28" type="DOUBLE"/>
<field name="actual_TCP_speed" type="VECTOR6D"/>
<field name="actual_TCP_pose" type="VECTOR6D"/>
<field name="actual_TCP_force" type="VECTOR6D"/>
<field name="actual_q" type="VECTOR6D"/>
<field name="actual_qd" type="VECTOR6D"/>
<field name="actual_current" type="VECTOR6D"/>
<field name="joint_control_output" type="VECTOR6D"/>
</recipe>
<recipe key="watchdog">
<field name="input_double_register_23" type="DOUBLE"/>
<field name="input_double_register_24" type="DOUBLE"/>
<field name="input_double_register_25" type="DOUBLE"/>
<field name="input_double_register_26" type="DOUBLE"/>
<field name="input_double_register_27" type="DOUBLE"/>
<field name="input_double_register_28" type="DOUBLE"/>
</recipe>
</rtde_config>
