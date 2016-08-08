# coding: utf-8
import matplotlib.pyplot as plt
import sys
from math import *
import numpy as np

offset_gps = -0.40
gps_local_noise = 0.4 * 0.4
gps_global_noise = 2.0 * 2.0

moy_visu_vel = 0.0
moy_meca_vel = 0.0
meca_cpt = 0
visu_cpt = 0
predicted_gps_x = []
predicted_gps_y = []

def printVariances():
	lst_gps_x = []
	lst_gps_y = []

	lst_meca_vel_x = []
	lst_visu_vel_x = []
	lst_visu_vel_y = []

	lst_yaw = []

	for i in range(len(dt)):
		if(meca_position_measure_received[i]):
			lst_meca_vel_x.append(meca_velocity_x[i])

		if(visu_velocity_measure_received[i]):
			lst_visu_vel_x.append(visu_velocity_x[i])
			lst_visu_vel_y.append(visu_velocity_y[i])

		if(gps_position_received[i]):
			lst_gps_x.append(gps_x[i])
			lst_gps_y.append(gps_y[i])

		if(imu_theta_received[i]):
			lst_yaw.append(imu_yaw_velocity[i])

	print "\nVariances:"
	print "gps x:", np.var(lst_gps_x)
	print "gps y:", np.var(lst_gps_y)
	print "meca vel x:", np.var(lst_meca_vel_x)
	print "visu vel x:", np.var(lst_visu_vel_x)
	print "visu vel y:", np.var(lst_visu_vel_y)
	print "theta vel:", np.var(lst_yaw)


# import simplekml
def resetKalman():
	global estimated_state_matrix_x_
	global predicted_state_matrix_x_
	global measurement_variable_matrix_y_
	global transition_matrix_f_
	global estimated_state_variance_matrix_p_
	global predicted_state_variance_matrix_p_
	global process_noise_cov_matrix_q_
	global measurement_noise_cov_matrix_r_
	global process_noise_variance_w_
	global jacobian_transition_matrix_f_
	global jacobian_measurement_matrix_h_



	estimated_state_matrix_x_ = np.matrix(np.zeros(10)).T

	predicted_state_matrix_x_ = np.matrix(np.zeros(10)).T

	estimated_state_variance_matrix_p_ =  np.matrix(np.zeros((10,10)))
	estimated_state_variance_matrix_p_[5,5] = pi * pi #ecart type au carré
	estimated_state_variance_matrix_p_[6,6] = 2.0 * 2.0 #ecart type au carré
	estimated_state_variance_matrix_p_[7,7] = 2.0 * 2.0 #ecart type au carré
	estimated_state_variance_matrix_p_[8,8] = 2.0 * 2.0
	estimated_state_variance_matrix_p_[9,9] = 2.0 * 2.0

	predicted_state_variance_matrix_p_ = np.matrix(np.zeros((10,10)))

	process_noise_variance_w_ = np.matrix(np.zeros((7,7)))
	process_noise_variance_w_[0,0] = 0.25 * 0.25     # σ²bvxr
	process_noise_variance_w_[1,1] = 0.01 * 0.01       # σ²bvyr
	process_noise_variance_w_[2,2] = 1.0 * 1.0       # σ²baxr
	process_noise_variance_w_[3,3] = 0.01 * 0.01       # σ²bayr
	process_noise_variance_w_[4,4] = 0.0015 * 0.0015 # σ²bωm
	process_noise_variance_w_[5,5] = 0.4 * 0.4
	process_noise_variance_w_[6,6] = 0.4 * 0.4

	process_noise_cov_matrix_q_ = np.matrix(np.zeros((10,10)))

	measurement_noise_cov_matrix_r_ = np.matrix(np.zeros((7,7)))
	measurement_noise_cov_matrix_r_[0,0] = gps_local_noise # σ²xgps
	measurement_noise_cov_matrix_r_[1,1] = gps_local_noise # σ²ygps
	measurement_noise_cov_matrix_r_[2,2] = 0.01 # σ²vxrOM
	measurement_noise_cov_matrix_r_[3,3] = 0.04 # σ²vxrOV
	measurement_noise_cov_matrix_r_[4,4] = 0.01 # σ²vyrOV
	measurement_noise_cov_matrix_r_[5,5] = gps_global_noise
	measurement_noise_cov_matrix_r_[6,6] = gps_global_noise

	jacobian_transition_matrix_f_ = np.matrix(np.eye(10))
	jacobian_measurement_matrix_h_ = np.matrix(np.zeros((7,10)))

#########################################################################
###							Plot parameters							  ###
#########################################################################
###   You can specify an input file to read."results.txt" is the      ###
###   default one.													  ###
###   Usage: python sensor_fusion_tuning.py input_file_path			  ###
#########################################################################
input_file_path = "file.csv"

if(len(sys.argv)>1):
	input_file_path = sys.argv[1]

############################################
###	  Récupération des mesures Excel	 ###
############################################
time, dt, imu_reseted, gps_position_received, initial_heading, gps_x, gps_y, om_gauche, om_droit, meca_position_measure_received, meca_fiab, meca_position_x, meca_position_y, meca_velocity_x, visu_position_measure_received, visu_fiab, visu_position_x, visu_position_y, visu_velocity_measure_received, visu_velocity_x, visu_velocity_y, imu_accel_measure_received, accel_mss_x, accel_mss_y, imu_theta_received, heading, imu_theta, imu_yaw_velocity, kalman_position_x, kalman_position_y = np.loadtxt(input_file_path, delimiter=';', unpack=True, skiprows=1)

abscisse = []
for i in range(len(time)):
	abscisse.append(i)

################################
###	  Variables globales	 ###
################################
kalman_x = []
kalman_y = []

########################
###	  Constantes	 ###
########################
epsilon = 0.0001

################################
###	  Kalman initialisation	 ###
################################
estimated_state_matrix_x_ = np.matrix(np.zeros(10)).T
predicted_state_matrix_x_ = np.matrix(np.zeros(10)).T
estimated_state_variance_matrix_p_ =  np.matrix(np.zeros((10,10)))
predicted_state_variance_matrix_p_ = np.matrix(np.zeros((10,10)))
resetKalman()

for i in range(len(time)):
	if imu_reseted[i]==1.0:
		last_heading = estimated_state_matrix_x_[5]
		resetKalman()
		estimated_state_matrix_x_[5] = last_heading

	####################################
	###	  Time update (Prediction)	 ###
	####################################
	predicted_state_f = np.matrix(np.zeros(10)).T
	predicted_state_f[0] =  estimated_state_matrix_x_[0] + dt[i] * estimated_state_matrix_x_[2] * cos(estimated_state_matrix_x_[4]) - dt[i] * estimated_state_matrix_x_[3] * sin(estimated_state_matrix_x_[4])
	predicted_state_f[1] =  estimated_state_matrix_x_[1] + dt[i] * estimated_state_matrix_x_[2] * sin(estimated_state_matrix_x_[4]) + dt[i] * estimated_state_matrix_x_[3] * cos(estimated_state_matrix_x_[4])


	if(om_gauche[i-1]!=None and om_droit[i-1]!=None):
		velocity_command_factor = 0.4 / 103
		previous_command = (om_gauche[i-1] + om_droit[i-1])/2
		current_command = (om_gauche[i] + om_droit[i])/2
		delta_command = current_command - previous_command

		if( previous_command * delta_command >= 0):
			predicted_state_f[2] = estimated_state_matrix_x_[2] + velocity_command_factor * delta_command
			jacobian_transition_matrix_f_[2,2] = 1.0

		else:
			predicted_state_f[2] = estimated_state_matrix_x_[2] * (previous_command + delta_command) / previous_command
			jacobian_transition_matrix_f_[2,2] = (previous_command + delta_command) / previous_command

	else:
		predicted_state_f[2] = 0.0
		jacobian_transition_matrix_f_[2,2] = 0.0

	predicted_state_f[3] =  estimated_state_matrix_x_[3] * 0.99
	predicted_state_f[4] =  estimated_state_matrix_x_[4] + dt[i] * imu_yaw_velocity[i]
	predicted_state_f[5] =  estimated_state_matrix_x_[5]
	predicted_state_f[6] =  estimated_state_matrix_x_[6]
	predicted_state_f[7] =  estimated_state_matrix_x_[7]
	predicted_state_f[8] =  estimated_state_matrix_x_[8]
	predicted_state_f[9] =  estimated_state_matrix_x_[9]

	jacobian_transition_matrix_f_[3,3] = 0.99

	jacobian_transition_matrix_f_[0,2] =  dt[i] * cos(estimated_state_matrix_x_[4])
	jacobian_transition_matrix_f_[0,3] =  -dt[i] * sin(estimated_state_matrix_x_[4])
	jacobian_transition_matrix_f_[0,4] =  -dt[i] * estimated_state_matrix_x_[2] * sin(estimated_state_matrix_x_[4]) - dt[i] * estimated_state_matrix_x_[3] * cos(estimated_state_matrix_x_[4])

	jacobian_transition_matrix_f_[1,2] =  dt[i] * sin(estimated_state_matrix_x_[4])
	jacobian_transition_matrix_f_[1,3] =  dt[i] * cos(estimated_state_matrix_x_[4])
	jacobian_transition_matrix_f_[1,4] =  dt[i] * estimated_state_matrix_x_[2] * cos(estimated_state_matrix_x_[4]) - dt[i] * estimated_state_matrix_x_[3] * sin(estimated_state_matrix_x_[4])

	jacobian_noise_matrix_g =  np.matrix(np.zeros((10,7)))
	jacobian_noise_matrix_g[0,0] = dt[i] * cos(estimated_state_matrix_x_[4])
	jacobian_noise_matrix_g[0,1] = -dt[i] * sin(estimated_state_matrix_x_[4])
	jacobian_noise_matrix_g[1,0] = dt[i] * sin(estimated_state_matrix_x_[4])
	jacobian_noise_matrix_g[1,1] = dt[i] * cos(estimated_state_matrix_x_[4])
	jacobian_noise_matrix_g[2,2] = dt[i]
	jacobian_noise_matrix_g[3,3] = dt[i]
	jacobian_noise_matrix_g[4,4] = dt[i]
	jacobian_noise_matrix_g[8,5] = dt[i]
	jacobian_noise_matrix_g[9,6] = dt[i]

	process_noise_cov_matrix_q_ = jacobian_noise_matrix_g * process_noise_variance_w_ * jacobian_noise_matrix_g.T

	predicted_state_matrix_x_ = np.matrix(predicted_state_f)

	predicted_state_variance_matrix_p_ = (jacobian_transition_matrix_f_ * estimated_state_variance_matrix_p_ * (jacobian_transition_matrix_f_.transpose())) + process_noise_cov_matrix_q_


	# print "estimated_state_matrix_x_"
	# print estimated_state_matrix_x_
	# print "predicted_state_matrix_x_"
	# print predicted_state_matrix_x_
	# print "estimated_state_variance_matrix_p_"
	# print estimated_state_variance_matrix_p_
	# print "predicted_state_variance_matrix_p_"
	# print predicted_state_variance_matrix_p_
	# print "jacobian_transition_matrix_f_"
	# print jacobian_transition_matrix_f_
	# print "process_noise_cov_matrix_q_"
	# print process_noise_cov_matrix_q_

	############################################
	###	  Measurement Update (Correction)	 ###
	############################################

	if(meca_position_measure_received[i]):
		if(meca_fiab[i]<0.001):
			meca_fiab[i] = 0.001
	else:
		meca_fiab[i] = 1.0

	if(visu_position_measure_received[i]):
		if(visu_fiab[i]<0.001):
			visu_fiab[i] = 0.001
	else:
		visu_fiab[i] = 1.0


	measurement_noise_cov_matrix_r_ = np.matrix(np.zeros((7,7)))
	measurement_noise_cov_matrix_r_[0,0] = gps_local_noise # σ²xgps
	measurement_noise_cov_matrix_r_[1,1] = gps_local_noise # σ²ygps
	measurement_noise_cov_matrix_r_[2,2] = 0.01 / (meca_fiab[i] * meca_fiab[i]) # σ²vxrOM
	measurement_noise_cov_matrix_r_[3,3] = 0.04 / ( visu_fiab[i] * visu_fiab[i])# σ²vxrOV
	measurement_noise_cov_matrix_r_[4,4] = 0.01 / ( visu_fiab[i] * visu_fiab[i])# σ²vyrOV
	measurement_noise_cov_matrix_r_[5,5] = gps_global_noise # σ²vyrOV
	measurement_noise_cov_matrix_r_[6,6] = gps_global_noise # σ²vyrOV



	measurement_matrix_y_ = np.matrix(np.zeros(7)).transpose()

	predicted_measurement_h = np.matrix(np.zeros(7)).transpose()

	# np.matrix(np.zeros((5,1)))
	predicted_measurement_h[0] = predicted_state_matrix_x_[0] * cos(predicted_state_matrix_x_[5]) - predicted_state_matrix_x_[1] * sin(predicted_state_matrix_x_[5]) + offset_gps*cos(predicted_state_matrix_x_[4] + predicted_state_matrix_x_[5]) + predicted_state_matrix_x_[6]
	predicted_measurement_h[1] = predicted_state_matrix_x_[0] * sin(predicted_state_matrix_x_[5]) + predicted_state_matrix_x_[1] * cos(predicted_state_matrix_x_[5]) + offset_gps*sin(predicted_state_matrix_x_[4] + predicted_state_matrix_x_[5]) + predicted_state_matrix_x_[7]
	predicted_measurement_h[2] = predicted_state_matrix_x_[2]
	predicted_measurement_h[3] = predicted_state_matrix_x_[2]
	predicted_measurement_h[4] = predicted_state_matrix_x_[3]
	predicted_measurement_h[5] = predicted_state_matrix_x_[8]
	predicted_measurement_h[6] = predicted_state_matrix_x_[9]


 	#  	Dérivée de h(X) selon X
	#                 x          y     vxr vyr  Θ             Θ₀
	# 	Hk+1 = / cos(θ₀) ; -sin(θ₀) ; 0 ; 0 ; 0 ; -x*sin(θ₀) - y*cos(θ₀) \
	# 	       | sin(θ₀) ;  cos(θ₀) ; 0 ; 0 ; 0 ;  x*cos(θ₀) - y*sin(θ₀) |
	# 	       | 0       ;  0       ; 1 ; 0 ; 0 ;  0                     |
	# 	       | 0       ;  0       ; 1 ; 0 ; 0 ;  0                     |
	# 	       \ 0       ;  0       ; 0 ; 1 ; 0 ;  0                     /

	jacobian_measurement_matrix_h_[0,0] = cos(predicted_state_matrix_x_[5])
	jacobian_measurement_matrix_h_[0,1] = -sin(predicted_state_matrix_x_[5])
	jacobian_measurement_matrix_h_[0,4] = -offset_gps*sin(predicted_state_matrix_x_[4]+predicted_state_matrix_x_[5])
	jacobian_measurement_matrix_h_[0,5] = -predicted_state_matrix_x_[0] * sin(predicted_state_matrix_x_[5]) - predicted_state_matrix_x_[1] * cos(predicted_state_matrix_x_[5]) - offset_gps*sin(predicted_state_matrix_x_[4]+predicted_state_matrix_x_[5])
	jacobian_measurement_matrix_h_[0,6] = 1.0
	jacobian_measurement_matrix_h_[0,8] = 1.0

	jacobian_measurement_matrix_h_[1,0] = sin(predicted_state_matrix_x_[5])
	jacobian_measurement_matrix_h_[1,1] = cos(predicted_state_matrix_x_[5])
	jacobian_measurement_matrix_h_[1,4] = offset_gps*cos(predicted_state_matrix_x_[4]+predicted_state_matrix_x_[5])
	jacobian_measurement_matrix_h_[1,5] = predicted_state_matrix_x_[0] * cos(predicted_state_matrix_x_[5]) - predicted_state_matrix_x_[1] * sin(predicted_state_matrix_x_[5]) + offset_gps*cos(predicted_state_matrix_x_[4]+predicted_state_matrix_x_[5])
	jacobian_measurement_matrix_h_[1,7] = 1.0
	jacobian_measurement_matrix_h_[1,9] = 1.0

	jacobian_measurement_matrix_h_[2,2] = 1.0
	jacobian_measurement_matrix_h_[3,2] = 1.0
	jacobian_measurement_matrix_h_[4,3] = 1.0
	jacobian_measurement_matrix_h_[5,8] = 1.0
	jacobian_measurement_matrix_h_[6,9] = 1.0



	if(om_gauche[i]==0 and om_droit[i]==0):
		gps_position_received[i] = False
		# visu_velocity_measure_received[i] = False

	if( not gps_position_received[i] ):
		jacobian_measurement_matrix_h_[0,0] = 0.0
		jacobian_measurement_matrix_h_[0,1] = 0.0
		jacobian_measurement_matrix_h_[0,5] = 0.0
		jacobian_measurement_matrix_h_[0,6] = 0.0
		jacobian_measurement_matrix_h_[0,7] = 0.0
		jacobian_measurement_matrix_h_[0,8] = 0.0

		jacobian_measurement_matrix_h_[1,0] = 0.0
		jacobian_measurement_matrix_h_[1,1] = 0.0
		jacobian_measurement_matrix_h_[1,5] = 0.0
		jacobian_measurement_matrix_h_[1,6] = 0.0
		jacobian_measurement_matrix_h_[1,7] = 0.0
		jacobian_measurement_matrix_h_[1,9] = 0.0

		jacobian_measurement_matrix_h_[5,8] = 0.0
		jacobian_measurement_matrix_h_[6,9] = 0.0

		predicted_measurement_h[0] = 0.0
		predicted_measurement_h[1] = 0.0

	else:
		measurement_matrix_y_[0] = gps_x[i]
		measurement_matrix_y_[1] = gps_y[i]
		predicted_gps_x.append(predicted_measurement_h.item(0))
		predicted_gps_y.append(predicted_measurement_h.item(1))

	# visu_position_measure_received[i] = False
	if( not visu_position_measure_received[i] ):
		jacobian_measurement_matrix_h_[3,2] = 0.0
		jacobian_measurement_matrix_h_[4,3] = 0.0

		predicted_measurement_h[3] = 0.0
		predicted_measurement_h[4] = 0.0
	else:
		measurement_matrix_y_[3] = visu_velocity_x[i]
		measurement_matrix_y_[4] = visu_velocity_y[i]
		moy_visu_vel = moy_visu_vel + abs(visu_velocity_x[i])
		visu_cpt = visu_cpt +1

	# meca_position_measure_received[i] = False
	if( not meca_position_measure_received[i] ):
		jacobian_measurement_matrix_h_[2,2] = 0.0

		predicted_measurement_h[2] = 0.0
	else:
		measurement_matrix_y_[2] = meca_velocity_x[i]
		moy_meca_vel = moy_meca_vel + abs(meca_velocity_x[i])
		meca_cpt = meca_cpt + 1

	#  υk+1 = Ym + h( Xk+1|k )
	innovation_matrix = measurement_matrix_y_ - predicted_measurement_h

	#  Sk+1 = Hk+1 * Pk+1|k * Hk+1t + Rk+1
	innovation_cov_matrix_s = (jacobian_measurement_matrix_h_ * predicted_state_variance_matrix_p_ * jacobian_measurement_matrix_h_.transpose()) + measurement_noise_cov_matrix_r_

	#  Kk+1 = Pk+1|k * Hk+1t * S⁻¹k+1
	kalman_gain_matrix_k = predicted_state_variance_matrix_p_ * jacobian_measurement_matrix_h_.transpose() * np.linalg.inv(innovation_cov_matrix_s)

	#  Xk+1|k+1 = Xk+1|k + Kk+1 * υk+1
	estimated_state_matrix_x_ = predicted_state_matrix_x_ + (kalman_gain_matrix_k * innovation_matrix)

	#  Pk+1|k+1 = (I - Kk+1 * Hk+1)*Pk+1|k
	estimated_state_variance_matrix_p_ = ( np.matrix(np.eye(10)) - (kalman_gain_matrix_k * jacobian_measurement_matrix_h_) ) * predicted_state_variance_matrix_p_

	###################
	###	  Affichage	###
	###################
	kalman_x.append( estimated_state_matrix_x_.item(0))
	kalman_y.append( estimated_state_matrix_x_.item(1))

	# if(time[i]>=14101057110031049):
	# print imu_theta[i]
	# print estimated_state_matrix_x_.item(4)
	# raw_input("...")

	# print estimated_state_matrix_x_.item(0), estimated_state_matrix_x_.item(1)
	# raw_input("...")

	# if(time[i]>=1410105717929103):
	#
	# 	print "dbt time " , int(time[i])
	# 	print "estimated_state_matrix_x_"
	# 	print estimated_state_matrix_x_
	# 	print "predicted_state_matrix_x_"
	# 	print predicted_state_matrix_x_
	# 	print "estimated_state_variance_matrix_p_"
	# 	print estimated_state_variance_matrix_p_
	# 	print "predicted_state_variance_matrix_p_"
	# 	print predicted_state_variance_matrix_p_
	# 	print "jacobian_transition_matrix_f_"
	# 	print jacobian_transition_matrix_f_
	# 	print "process_noise_cov_matrix_q_"
	# 	print process_noise_cov_matrix_q_
	#
	#
	# 	print "innovation_cov_matrix_s"
	# 	print innovation_cov_matrix_s
	# 	print "inv innovation_cov_matrix_s"
	# 	# np.set_printoptions(precision=3)
	# 	print '%.10f' % np.linalg.inv(innovation_cov_matrix_s)[2,2]
	#
	# 	print np.linalg.inv(innovation_cov_matrix_s)
	# 	print "kalman_gain_matrix_k"
	# 	print kalman_gain_matrix_k
	# 	print "estimated_state_matrix_x_"
	# 	print estimated_state_matrix_x_
	# 	print "estimated_state_variance_matrix_p_"
	# 	print estimated_state_variance_matrix_p_
	# 	print "innovation_matrix"
	# 	print innovation_matrix
	# 	print "measurement_matrix_y_"
	# 	print measurement_matrix_y_
	# 	print "fin time " , int(time[i])
	# 	raw_input("...")

	# print "innovation_cov_matrix_s"
	# print innovation_cov_matrix_s
	# print "inv innovation_cov_matrix_s"
	# # np.set_printoptions(precision=3)
	# print '%.10f' % np.linalg.inv(innovation_cov_matrix_s)[2,2]
	#
	# print np.linalg.inv(innovation_cov_matrix_s)
	# print "kalman_gain_matrix_k"
	# print kalman_gain_matrix_k
	# print "estimated_state_matrix_x_"
	# print estimated_state_matrix_x_
	# print "estimated_state_variance_matrix_p_"
	# print estimated_state_variance_matrix_p_
	# print "innovation_matrix"
	# print innovation_matrix
	# print "measurement_matrix_y_"
	# print measurement_matrix_y_
	#
	# raw_input("continuer...")


	# print "x:",estimated_state_matrix_x_[6,0] , " -> ", estimated_state_variance_matrix_p_[6,6]
	# print "y:",estimated_state_matrix_x_[7,0] , " -> ", estimated_state_variance_matrix_p_[7,7]

# print "moy vel visu", moy_visu_vel/visu_cpt
# print "moy vel meca", moy_meca_vel/meca_cpt
# printVariances()

x_tempo = gps_x
y_tempo = gps_y
gps_x = []
gps_y = []

for i in range(len(dt)):
	if(gps_position_received[i]==1):
		gps_x.append(x_tempo[i])
		gps_y.append(y_tempo[i])



fig1 = plt.figure()
ax1 = fig1.add_subplot(111)
plt.axis('equal')
fig1.canvas.set_window_title('Positions')
plt.gca().invert_xaxis()
plt.plot(kalman_y, kalman_x, 'rx-', label="kalman")
plt.plot(meca_position_y, meca_position_x, 'gx-', label="meca")
plt.plot(visu_position_y, visu_position_x, 'bx-', label="visu")
plt.plot(predicted_gps_y, predicted_gps_x, 'ko-', label="gps predicted")

plt.plot(gps_y, gps_x, 'yo-', label="gps")
# plt.plot(prediction_y, prediction_x, 'kx-', label="prediction")

plt.legend(loc='best')
# cpt = 100
# for i in range(len(dt)):
# 	if(cpt==100):
# 		# ax1.annotate( str(degrees(heading[i])), xy=(kalman_y[i], kalman_x[i]), xycoords='data',xytext=(kalman_y[i]+0.02, kalman_x[i]+0.02), arrowprops=dict(facecolor='black', shrink=0.05, width=1))
# 		cpt = 0
# 	cpt = cpt+1



fig2 = plt.figure()
plt.axis('equal')
fig2.canvas.set_window_title('Kalman robot VS Kalman simulateur')
plt.gca().invert_xaxis()
plt.plot(kalman_position_y, kalman_position_x, 'gx-', label="Kalman robot")
plt.plot(kalman_y, kalman_x, 'bx-', label="Kalman simulateur")
plt.legend(loc='best')




plt.show()
