# coding: utf-8
import matplotlib.pyplot as plt
import sys
from math import *
import numpy as np
# import simplekml

def maximum(liste):
	max = -1
	for x in liste:
		if(abs(x)>max):
			max = abs(x)
	return max

def resetKalman():
	global state_matrix_x_
	global temp_state_matrix_x_
	global prev_state_matrix_x_
	global measurement_variable_matrix_y_
	global transition_matrix_f_
	global state_variance_matrix_p_
	global temp_state_variance_matrix_p_
	global prev_state_variance_matrix_p_
	global process_noise_cov_matrix_q_
	global measurement_matrix_h_
	global observation_measurement_noise_cov_matrix_r_


	state_matrix_x_ = np.matrix(np.zeros(7)).T	  # X estimated state
	temp_state_matrix_x_ = np.matrix(np.zeros(7)).T # Xt|t-1
	prev_state_matrix_x_ = np.matrix(np.zeros(7)).T # Xt-1|t-1
	measurement_variable_matrix_y_ = np.matrix(np.zeros(7)).T # Y matrix  : sensors measurement ( often known as Z )

	transition_matrix_f_ = np.matrix(np.eye(7))		# F transition matrix between states

	state_variance_matrix_p_ =  np.matrix(np.zeros((7,7)))	 # P estimation error
	temp_state_variance_matrix_p_ = np.matrix(np.zeros((7,7))) # Pt|t-1
	prev_state_variance_matrix_p_ = np.matrix(np.zeros((7,7))) # Pt-1|t-1
	prev_state_variance_matrix_p_[0,0] = 0.00001
	prev_state_variance_matrix_p_[1,1] = 0.00001
	prev_state_variance_matrix_p_[2,2] = 0.00001
	prev_state_variance_matrix_p_[3,3] = 0.00001
	prev_state_variance_matrix_p_[4,4] = 0.00001
	prev_state_variance_matrix_p_[5,5] = 0.00001
	prev_state_variance_matrix_p_[6,6] = 0.00001

	process_noise_cov_matrix_q_ = np.matrix(np.eye(7)) # Q error due to process
	process_noise_cov_matrix_q_[0,0] = 0.0001 #process_sd_meca_x * process_sd_meca_x
	process_noise_cov_matrix_q_[1,1] = 0.0001 #process_sd_meca_y * process_sd_meca_y
	process_noise_cov_matrix_q_[2,2] = 0.0001 #process_sd_visu_velocity_x * process_sd_visu_velocity_x
	process_noise_cov_matrix_q_[3,3] = 0.0001 #process_sd_visu_velocity_y * process_sd_visu_velocity_y
	process_noise_cov_matrix_q_[4,4] = 0.0001 #process_sd_imu_accel_x * process_sd_imu_accel_x
	process_noise_cov_matrix_q_[5,5] = 0.0001 #process_sd_imu_accel_y * process_sd_imu_accel_y
	process_noise_cov_matrix_q_[6,6] = 0.0001 #process_sd_imu_theta * process_sd_imu_theta

	measurement_matrix_h_ = np.matrix(np.eye(7))	   # H observation model
	# observation_measurement_noise_cov_matrix_r_ = np.matrix(np.eye(7)) # R measurement noise
	# observation_measurement_noise_cov_matrix_r_[0,0] = 0.05 #sd_meca_x
	# observation_measurement_noise_cov_matrix_r_[1,1] = 0.05 #sd_meca_y
	# observation_measurement_noise_cov_matrix_r_[2,2] = 0.004# sd_visu_velocity_x
	# observation_measurement_noise_cov_matrix_r_[3,3] = 0.0004# sd_visu_velocity_y
	# observation_measurement_noise_cov_matrix_r_[4,4] = 4.0# sd_imu_accel_x
	# observation_measurement_noise_cov_matrix_r_[5,5] = 1.5# sd_imu_accel_y
	# observation_measurement_noise_cov_matrix_r_[6,6] = 0.02# sd_imu_theta
	# observation_measurement_noise_cov_matrix_r_ = np.matrix(np.eye(7)) # R measurement noise
	# observation_measurement_noise_cov_matrix_r_[0,0] = 5.0 #sd_meca_x
	# observation_measurement_noise_cov_matrix_r_[1,1] = 5.0 #sd_meca_y
	# observation_measurement_noise_cov_matrix_r_[2,2] = 0.0031*0.0031 # sd_visu_velocity_x
	# observation_measurement_noise_cov_matrix_r_[3,3] = 0.0001*0.0001 # sd_visu_velocity_y
	# observation_measurement_noise_cov_matrix_r_[4,4] = 2.1146*2.1146*2.0 # sd_imu_accel_x
	# observation_measurement_noise_cov_matrix_r_[5,5] = 0.7527*0.7527*2.0 # sd_imu_accel_y
	# observation_measurement_noise_cov_matrix_r_[6,6] = 0.5777746718*0.5777746718 # sd_imu_theta
	observation_measurement_noise_cov_matrix_r_ = np.matrix(np.eye(7)) # R measurement noise
	observation_measurement_noise_cov_matrix_r_[0,0] = 5.0 #sd_meca_x
	observation_measurement_noise_cov_matrix_r_[1,1] = 5.0 #sd_meca_y
	observation_measurement_noise_cov_matrix_r_[2,2] = 0.0015*0.0015 # sd_visu_velocity_x
	observation_measurement_noise_cov_matrix_r_[3,3] = 0.0015*0.0015 # sd_visu_velocity_y
	observation_measurement_noise_cov_matrix_r_[4,4] = 1.5*1.5*1.0 # sd_imu_accel_x
	observation_measurement_noise_cov_matrix_r_[5,5] = 1.5*1.5*1.0 # sd_imu_accel_y
	observation_measurement_noise_cov_matrix_r_[6,6] = 0.5777746718*0.5777746718 # sd_imu_theta

HEADER = '\033[95m'
OKBLUE = '\033[94m'
OKGREEN = '\033[92m'
WARNING = '\033[93m'
FAIL = '\033[91m'
ENDC = '\033[0m'
BOLD = '\033[1m'
UNDERLINE = '\033[4m'

###############################################################################
###							Plot parameters							  ###
###############################################################################
###   You can specify an input file to read."results.txt" is the			###
###   default one.														  ###
###   Usage: python sensor_fusion_tuning.py input_file_path				 ###
###############################################################################
input_file_path = "file.csv"

if(len(sys.argv)>1):
	input_file_path = sys.argv[1]

###############################################
###	  Récupération des mesures Excel	 ###
###############################################
time, dt, imu_reseted, meca_position_measure_received, meca_position_x, meca_position_y, visu_position_measure_received, visu_position_x, visu_position_y, visu_velocity_measure_received, visu_velocity_x, visu_velocity_y, imu_accel_measure_received, accel_mss_x, accel_mss_y, imu_theta_measure_received, imu_theta, kalman_position_x, kalman_position_y = np.loadtxt(input_file_path, delimiter=';', unpack=True, skiprows=1)
# print np.var(visu_velocity_x)
# print np.var(visu_velocity_y)
#
# print np.var(accel_mss_x)
# print np.var(accel_mss_y)

meca_cov_pos_x = []
meca_cov_pos_y = []

for i in range( len(meca_position_x) ):
	if( meca_position_measure_received[i] == 1 ):
		meca_cov_pos_x.append( meca_position_x[i] )
		meca_cov_pos_y.append( meca_position_y[i] )


meca_cov_pos_dx = []
meca_cov_pos_dy = []

for i in range( len(meca_cov_pos_x) - 1 ):
	meca_cov_pos_dx.append( meca_cov_pos_x[i+1] - meca_cov_pos_x[i] )
	meca_cov_pos_dy.append( meca_cov_pos_y[i+1] - meca_cov_pos_y[i] )

# print meca_cov_pos_dx
#
# print np.var(meca_cov_pos_dx)
# print np.var(meca_cov_pos_dy)


# print np.var(visu_velocity_x)
# print np.var(visu_velocity_y)

abscisse = []
for i in range(len(time)):
	abscisse.append(i)
################################
###	  Variables globales	 ###
################################
kalman_x = []
kalman_y = []

prediction_x = []
prediction_y = []

###########################
###	  Constantes	 ###
###########################
epsilon = 0.0001

# PROCESS NOISE
# process_sd_meca_x = 0.18
# process_sd_meca_y = 0.18
#
# process_sd_visu_velocity_x = 0.80
# process_sd_visu_velocity_y = 0.74
#
# process_sd_imu_accel_x = 12.62*100
# process_sd_imu_accel_y = 10.86*100
#
# process_sd_imu_theta = 1.72

process_sd_meca_x = maximum(meca_cov_pos_dx)
process_sd_meca_y = maximum(meca_cov_pos_dy)

process_sd_visu_velocity_x = maximum(visu_velocity_x)
process_sd_visu_velocity_y = maximum(visu_velocity_y)

process_sd_imu_accel_x = maximum(accel_mss_x)
process_sd_imu_accel_y = maximum(accel_mss_y)

process_sd_imu_theta = maximum(imu_theta)

# SENSOR NOISE
sd_meca_x = np.var(meca_cov_pos_dx)
sd_meca_y = np.var(meca_cov_pos_dy)

sd_visu_velocity_x = np.var(visu_velocity_x)
sd_visu_velocity_y = np.var(visu_velocity_y)

sd_imu_accel_x = np.var(accel_mss_x)
sd_imu_accel_y = np.var(accel_mss_y)

sd_imu_theta = np.var(imu_theta)

# print "### PROCESS NOISE ###"
# print "process_sd_meca_x:", process_sd_meca_x
# print "process_sd_meca_y:", process_sd_meca_y
#
# print "process_sd_visu_velocity_x:", process_sd_visu_velocity_x
# print "process_sd_visu_velocity_y:", process_sd_visu_velocity_y
#
# print "process_sd_imu_accel_x:",process_sd_imu_accel_x
# print "process_sd_imu_accel_y:",process_sd_imu_accel_y
#
# print "process_sd_imu_theta:",process_sd_imu_theta
#
# print "\n### SENSOR NOISE ###"
# print "sd_meca_x:", sd_meca_x
# print "sd_meca_y:", sd_meca_y
#
# print "sd_visu_velocity_x:", sd_visu_velocity_x
# print "sd_visu_velocity_y:", sd_visu_velocity_y
#
# print "sd_imu_accel_x:", sd_imu_accel_x
# print "sd_imu_accel_y:", sd_imu_accel_y
#
# print "sd_imu_theta:", sd_imu_theta
######################################
###	  Kalman initialisation	 ###
######################################

state_matrix_x_ = np.matrix(np.zeros(7)).T	  # X estimated state
temp_state_matrix_x_ = np.matrix(np.zeros(7)).T # Xt|t-1
prev_state_matrix_x_ = np.matrix(np.zeros(7)).T # Xt-1|t-1
measurement_variable_matrix_y_ = np.matrix(np.zeros(7)).T # Y matrix  : sensors measurement ( often known as Z )

transition_matrix_f_ = np.matrix(np.eye(7))		# F transition matrix between states

state_variance_matrix_p_ =  np.matrix(np.zeros((7,7)))	 # P estimation error
temp_state_variance_matrix_p_ = np.matrix(np.zeros((7,7))) # Pt|t-1
prev_state_variance_matrix_p_ = np.matrix(np.zeros((7,7))) # Pt-1|t-1


process_noise_cov_matrix_q_ = np.matrix(np.eye(7)) # Q error due to process
observation_measurement_noise_cov_matrix_r_ = np.matrix(np.eye(7)) # R measurement noise


measurement_matrix_h_ = np.matrix(np.eye(7))	   # H observation model
resetKalman()

# print observation_measurement_noise_cov_matrix_r_
# print process_noise_cov_matrix_q_
# print measurement_matrix_h_
# print prev_state_variance_matrix_p_
# print prev_state_matrix_x_

for i in range(len(time)):

	if imu_reseted[i]==1.0:
		resetKalman()
	####################################
	###	  Time update (Prediction)	 ###
	####################################
	transition_matrix_f_[0,2] = dt[i]  # velocity_x applied to x

	transition_matrix_f_[0,4] = 0.5 * dt[i] * dt[i] * state_matrix_x_.item(5) * np.sin( state_matrix_x_.item(6) ) # accel_x applied to x

	transition_matrix_f_[1,3] = dt[i] # velocity_y applied to y

	transition_matrix_f_[1,5] = 0.5 * dt[i] * dt[i] * state_matrix_x_.item(4) * np.cos( state_matrix_x_.item(6) ) # accel_y applied to y

	transition_matrix_f_[2,4] = dt[i]  # applies to velocity_x predict

	transition_matrix_f_[3,5] = dt[i] # applies to velocity_y predict

	temp_state_matrix_x_ = transition_matrix_f_ * prev_state_matrix_x_

	temp_state_variance_matrix_p_ = ( transition_matrix_f_ * prev_state_variance_matrix_p_ * transition_matrix_f_.T ) + process_noise_cov_matrix_q_


	prediction_x.append(temp_state_matrix_x_.item(0))
	prediction_y.append(temp_state_matrix_x_.item(1))


	################################################
	###	  Measurement Update (Correction)	 ###
	################################################
	measurement_variable_matrix_y_[0] = meca_position_x[i]
	measurement_variable_matrix_y_[1] = meca_position_y[i]
	measurement_variable_matrix_y_[2] = visu_velocity_x[i]
	measurement_variable_matrix_y_[3] = visu_velocity_y[i]
	measurement_variable_matrix_y_[4] = accel_mss_x[i]
	measurement_variable_matrix_y_[5] = accel_mss_y[i]
	measurement_variable_matrix_y_[6] = imu_theta[i]


	if( meca_position_measure_received[i] ):
		measurement_matrix_h_[0,0] = 1.0
		measurement_matrix_h_[1,1] = 1.0
	else:
		measurement_matrix_h_[0,0] = 0.0
		measurement_matrix_h_[1,1] = 0.0

	if( visu_velocity_measure_received[i] ):
		measurement_matrix_h_[2,2] = 1.0
		measurement_matrix_h_[3,3] = 1.0
	else:
		measurement_matrix_h_[2,2] = 0.0
		measurement_matrix_h_[3,3] = 0.0

	if( imu_accel_measure_received[i] ):
		measurement_matrix_h_[4,4] = 1.0
		measurement_matrix_h_[5,5] = 1.0
	else:
		measurement_matrix_h_[4,4] = 0.0
		measurement_matrix_h_[5,5] = 0.0

	if( imu_theta_measure_received[i] ):
		measurement_matrix_h_[6,6] = 1.0
	else:
		measurement_matrix_h_[6,6] = 0.0


	matrix_innov =np.linalg.inv( ( measurement_matrix_h_ * temp_state_variance_matrix_p_ * measurement_matrix_h_.T ) + observation_measurement_noise_cov_matrix_r_ )
	matrix_k = temp_state_variance_matrix_p_ * measurement_matrix_h_.T * matrix_innov
	state_variance_matrix_p_ = ( np.matrix(np.eye(7)) - ( matrix_k * measurement_matrix_h_ ) ) * temp_state_variance_matrix_p_
	state_matrix_x_ = temp_state_matrix_x_ + ( matrix_k * ( measurement_variable_matrix_y_ - ( measurement_matrix_h_ * temp_state_matrix_x_ ) ) )
	prev_state_matrix_x_ = state_matrix_x_
	prev_state_variance_matrix_p_ = state_variance_matrix_p_


	###################
	###	  Affichage	###
	###################
	kalman_x.append( state_matrix_x_.item(0))
	kalman_y.append( state_matrix_x_.item(1))
	# print state_matrix_x_.item(0), state_matrix_x_.item(1)
	# raw_input("...")


fig1 = plt.figure()
ax1 = fig1.add_subplot(111)
plt.axis('equal')
fig1.canvas.set_window_title('Positions')
plt.gca().invert_xaxis()
plt.plot(kalman_y, kalman_x, 'rx-', label="kalman")
plt.plot(meca_position_y, meca_position_x, 'gx-', label="meca")
plt.plot(visu_position_y, visu_position_x, 'bx-', label="visu")
# plt.plot(prediction_y, prediction_x, 'kx-', label="prediction")

plt.legend(loc='best')
cpt = 100
for i in range(len(dt)):
	if(cpt==100):
		# ax1.annotate( str(degrees(imu_theta[i])), xy=(kalman_y[i], kalman_x[i]), xycoords='data',xytext=(kalman_y[i]+0.02, kalman_x[i]+0.02), arrowprops=dict(facecolor='black', shrink=0.05, width=1))
		cpt = 0
	cpt = cpt+1

# fig2 = plt.figure()
# plt.axis('equal')
# fig2.canvas.set_window_title('Accelerometers')
# plt.gca().invert_xaxis()
# plt.plot(accel_mss_y, accel_mss_x, 'bx')
#
# fig3 = plt.figure()
# plt.axis('equal')
# fig3.canvas.set_window_title('Visu speed')
# plt.gca().invert_xaxis()
# plt.plot(visu_velocity_y, visu_velocity_x, 'bx')
#
# fig4 = plt.figure()
# plt.axis('equal')
# fig4.canvas.set_window_title('Dt')
# plt.gca().invert_xaxis()
# plt.plot(abscisse, dt, 'bx')

fig5 = plt.figure()
plt.axis('equal')
fig5.canvas.set_window_title('Kalman robot VS Kalman simulateur')
plt.gca().invert_xaxis()
plt.plot(kalman_position_y, kalman_position_x, 'gx-', label="Kalman robot")
plt.plot(kalman_y, kalman_x, 'bx-', label="Kalman simulateur")
plt.legend(loc='best')

# fig6 = plt.figure()
# plt.axis('equal')
# fig6.canvas.set_window_title('accel_mss')
# plt.gca().invert_xaxis()
# plt.plot(accel_mss_y, accel_mss_x, 'gx-', label="accelero")
# plt.legend(loc='best')

plt.show()
#
# while i<len(dt):
#
#     # plt.plot(visu_x[0:i], visu_y[0:i], 'bx-', label="visu")
#     # plt.plot(meca_x[0:i], meca_y[0:i], 'gx-', label="meca")
#     # plt.plot(kalman_x[0:i], kalman_y[0:i], 'rx-', label="kalman")
#     plt.plot(visu_y_raw[i], visu_x_raw[i], 'bx-', label="visu_raw")
#     ax1.annotate( str(i)+':'+str(angles_visu[i]), xy=(visu_y_raw[i], visu_x_raw[i]), xycoords='data' )
#     #ax.annotate('azer', xy=(visu_y_raw[i], visu_x_raw[i] ), xytext=(visu_y_raw[i], visu_x_raw[i]), xycoords='data' )
#
#
#
#     plt.plot(meca_y_raw[i], meca_x_raw[i], 'gx-', label="meca_raw")
#     ax2.annotate( str(i)+':'+str(angles_meca[i]), xy=(meca_y_raw[i], meca_x_raw[i]), xycoords='data' )
#
#     plt.plot(kalman_position_y[i], kalman_position_x[i], 'rx-', label="kalman_semi_raw")
#
#     ax3.annotate( str(i)+':'+str(angles_kalman[i]), xy=(kalman_position_y[i], kalman_position_x[i]), xycoords='data' )
#
#     plt.draw()
#     print "\n+------------------------------------------------------------------------------+"
#     print "                          position %d at t = %4.3f s" % (i,time_elapsed[i])
#     print "+------------------------------------------------------------------------------+\n"
#     print "\t[ Angle global rad/deg","\t;\tDelta angle rad/deg",";\tDelta distance ]"
#     print FAIL + "Kalman\t[ %2.5f/%2.5f\t;\t%2.5f/%2.5f\t;\t%2.5f ]" %(angles_kalman[i], degrees(angles_kalman[i]), kalman_delta_angle[i],degrees(kalman_delta_angle[i]),kalman_delta_distance[i]) + ENDC
#     print OKBLUE + "Visu\t[ %2.5f/%2.5f\t;\t%2.5f/%2.5f\t;\t%2.5f ]" %(angles_visu[i], degrees(angles_visu[i]), visu_delta_angle[i],degrees(visu_delta_angle[i]),visu_delta_distance[i]) + ENDC
#     print OKGREEN + "Meca\t[ %2.5f/%2.5f\t;\t%2.5f/%2.5f\t;\t%2.5f ]" %(angles_meca[i], degrees(angles_meca[i]), meca_delta_angle[i],degrees(meca_delta_angle[i]),meca_delta_distance[i]) + ENDC
#     print "\n==>\t[ fiab_meca",fiab_meca[i],"\t;\tfiab_visu",fiab_visu[i],"]\n"
#
#     print WARNING+UNDERLINE+"Remarques:"+ENDC+WARNING
#     if i in commentaires.keys():
#         print commentaires[i]+ENDC
#     else:
#         print "Aucune\n"+ENDC
#
#     res = raw_input("[Entree] -> Point suivant ; [P] -> Point précédent ; [numéro position] -> Position particulière ; [A] -> Tout afficher ; [Q] -> Quitter: ")
#
#     if res=="a" or res=="A":
#         # display = True
#         i = len(kalman_x)-1
#         for()
#         # plt.clf()
#         # plt.close()
#     elif res.isdigit():
#         i = eval(res)
#
#     elif res=="q" or res=="Q":
#         break
#     elif res=="p" or res=="P":
#         i = i-1
#     else:
#         i = i+1
