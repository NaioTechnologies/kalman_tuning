import csv
import sys

input_file_path = "log_lvl2"

if(len(sys.argv)>1):
    input_file_path = sys.argv[1]

f = open(input_file_path,"r")


imu_reseted = 0.0
time = 0.0
dt = 0.0
meca_position_measure_received = 0.0
meca_position_x = 0.0
meca_position_y = 0.0
visu_velocity_measure_received = 0.0
visu_velocity_x = 0.0
visu_velocity_y = 0.0
imu_accel_measure_received = 0.0
accel_mss_x = 0.0
accel_mss_y = 0.0
imu_theta_measure_received = 0.0
imu_theta = 0.0
visu_position_measure_received = 0.0
visu_position_x = 0.0
visu_position_y = 0.0
kalman_position_x = 0.0
kalman_position_y = 0.0

with open('file.csv', 'wb') as csvfile:
    spamwriter = csv.writer(csvfile, delimiter=';', quotechar='|', quoting=csv.QUOTE_MINIMAL)
    spamwriter.writerow(['time'] + ['dt'] + ['imu_reseted'] + ['meca_position_measure_received'] + ['meca_position_x'] + ['meca_position_y'] + ['visu_position_measure_received'] + ['visu_position_x'] + ['visu_position_y'] + ['visu_velocity_measure_received'] + ['visu_velocity_x'] + ['visu_velocity_y'] + ['imu_accel_measure_received'] + ['accel_mss_x'] + ['accel_mss_y'] + ['imu_theta_measure_received'] + ['imu_theta'] + ['kalman_position_x']+ ['kalman_position_y'])

    all_measures_read = False
    for line in f.readlines():

        if(all_measures_read or imu_reseted==1.0):
            spamwriter.writerow([time] + [dt] + [imu_reseted] + [meca_position_measure_received] + [meca_position_x] + [meca_position_y] + [visu_position_measure_received] + [visu_position_x] + [visu_position_y] + [visu_velocity_measure_received] + [visu_velocity_x] + [visu_velocity_y] + [imu_accel_measure_received] + [accel_mss_x] + [accel_mss_y] + [imu_theta_measure_received] + [imu_theta] + [kalman_position_x] + [kalman_position_y])
            imu_reseted = 0.0
            all_measures_read = False

        if ("SensorFusion::imuReseted :" in line):
            imu_reseted = 1.0

        if ("kalman time" in line):
            time = eval(line.split(" ")[-1])

        if ("kalman dt" in line):
            dt = eval(line.split(" ")[-1])

        if ("meca_position.x" in line):
            meca_position_measure_received = eval(line.split(" ")[4])
            meca_position_x = eval(line.split(" ")[-1])

        if ("meca_position.y" in line):
            meca_position_y = eval(line.split(" ")[-1])

        if ("visu_velocity_x" in line):
            visu_velocity_measure_received = eval(line.split(" ")[4])
            visu_velocity_x = eval(line.split(" ")[-1])

        if ("visu_velocity_y" in line):
            visu_velocity_y = eval(line.split(" ")[-1])

        if ("accel_mss_x" in line and "kalman" in line):
            imu_accel_measure_received = eval(line.split(" ")[4])
            accel_mss_x = eval(line.split(" ")[-1])

        if ("accel_mss_y" in line and "kalman" in line):
            accel_mss_y = eval(line.split(" ")[-1])

        if ("imu_theta" in line):
            imu_theta_measure_received = eval(line.split(" ")[4])
            imu_theta = eval(line.split(" ")[-1])

        if ("kalman visu position" in line):
            visu_position_measure_received = eval(line.split(" ")[6])
            visu_position_x = eval(line.split(" ")[-4])
            visu_position_y = eval(line.split(" ")[-2])

        if ("kalman kalman position :" in line):
            kalman_position_x = eval(line.split(" ")[-4])
            kalman_position_y = eval(line.split(" ")[-2])
            all_measures_read = True
