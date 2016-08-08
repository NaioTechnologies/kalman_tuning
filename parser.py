import csv
import sys

input_file_path = "log_lvl2"

if(len(sys.argv)>1):
    input_file_path = sys.argv[1]

f = open(input_file_path,"r")


imu_reseted = 0.0

with open('file.csv', 'wb') as csvfile:
    spamwriter = csv.writer(csvfile, delimiter=';', quotechar='|', quoting=csv.QUOTE_MINIMAL)
    # spamwriter.writerow(['time'] + ['dt'] + ['imu_reseted'] + ['gps_position_received'] + ['initial_heading'] + ['gps_x'] + ['gps_y'] + ['om_gauche'] + ['om_droit'] + ['meca_position_measure_received'] + ['meca_fiab'] + ['meca_position_x'] + ['meca_position_y'] + ['meca_velocity_x'] + ['visu_position_measure_received'] + ['visu_fiab'] + ['visu_position_x'] + ['visu_position_y'] + ['visu_velocity_measure_received'] + ['visu_velocity_x'] + ['visu_velocity_y'] + ['imu_accel_measure_received'] + ['accel_mss_x'] + ['accel_mss_y'] + ['imu_theta_received'] + ['heading'] + ['imu_theta'] + ['imu_yaw_velocity'] + ['kalman_position_x']+ ['kalman_position_y'])
    spamwriter.writerow(['time'] + ['dt'] + ['imu_reseted'] + ['gps_position_received'] + ['initial_heading'] + ['gps_x'] + ['gps_y'] + ['om_gauche'] + ['om_droit'] + ['meca_position_measure_received'] + ['meca_fiab'] + ['meca_position_x'] + ['meca_position_y'] + ['meca_velocity_x'] + ['visu_position_measure_received'] + ['visu_fiab'] + ['visu_position_x'] + ['visu_position_y'] + ['visu_velocity_measure_received'] + ['visu_velocity_x'] + ['visu_velocity_y'] + ['imu_accel_measure_received'] + ['accel_mss_x'] + ['accel_mss_y'] + ['imu_theta_received'] + ['imu_theta'] + ['imu_yaw_velocity'] + ['kalman_position_x']+ ['kalman_position_y'])

    all_measures_read = False
    imu_theta_received = 0.0
    imu_theta = 0.0
    visu_position_measure_received = 0.0
    visu_position_x = 0.0
    visu_position_y = 0.0
    kalman_position_x = 0.0
    kalman_position_y = 0.0
    imu_yaw_velocity = 0.0
    meca_velocity_x = 0.0
    om_gauche = 0.0
    om_droit = 0.0
    imu_accel_measure_received = 0.0
    accel_mss_x = 0.0
    accel_mss_y = 0.0

    for line in f.readlines():

        if(all_measures_read or imu_reseted==1.0):
            spamwriter.writerow([time] + [dt] + [imu_reseted] + [gps_position_received] + [initial_heading] + [gps_x] + [gps_y] + [om_gauche] + [om_droit] + [meca_position_measure_received] + [meca_fiab] + [meca_position_x] + [meca_position_y] + [meca_velocity_x] + [visu_position_measure_received] + [visu_fiab] + [visu_position_x] + [visu_position_y] + [visu_velocity_measure_received] + [visu_velocity_x] + [visu_velocity_y] + [imu_accel_measure_received] + [accel_mss_x] + [accel_mss_y] + [imu_theta_received] + [imu_theta] + [imu_yaw_velocity] + [kalman_position_x] + [kalman_position_y])
            imu_reseted = 0.0
            all_measures_read = False

        if ("SensorFusion::imuReseted :" in line):
            imu_reseted = 1.0

        if ("SensorFusion ORDRE_MOTEUR" in line):
            om_gauche = eval(line.split(" ")[-8])
            om_droit = eval(line.split(" ")[-6])

        if ("kalman time" in line):
            time = eval(line.split(" ")[-1])

        if ("kalman dt" in line):
            dt = eval(line.split(" ")[-1])

        if ("gps_position.x" in line):
            gps_position_received = eval(line.split(" ")[4])
            gps_x = eval(line.split(" ")[-1])

        if ("gps_position.y" in line):
            gps_y = eval(line.split(" ")[-1])

        if ("meca_position.x" in line):
            meca_position_measure_received = eval(line.split(" ")[4])
            meca_position_x = eval(line.split(" ")[-1])

        if ("meca_position.y" in line):
            meca_position_y = eval(line.split(" ")[-1])

        if ("meca_velocity_x" in line):
            meca_velocity_x = eval(line.split(" ")[-1])

        if ("kalman meca position" in line):
            meca_fiab = eval(line.split(" ")[-1])

        if ("kalman visu position" in line):
            visu_fiab = eval(line.split(" ")[-1])


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

        # if ("reconstructed_heading" in line):
        #     # reconstructed_heading_received = eval(line.split(" ")[4])
        #     heading = eval(line.split(" ")[-1])

        if ("imu_theta" in line):
            imu_theta_received = eval(line.split(" ")[4])
            imu_theta = eval(line.split(" ")[-1])

        if("imu_yaw_velocity" in line):
            imu_yaw_velocity = eval(line.split(" ")[-1])

        if ("kalman visu position" in line):
            visu_position_measure_received = eval(line.split(" ")[6])
            visu_position_x = eval(line.split(" ")[-4])
            visu_position_y = eval(line.split(" ")[-2])

        if ("kalman gps reference initial heading" in line):
            initial_heading = eval(line.split(" ")[-1])

        if ("kalman kalman position :" in line):
            kalman_position_x = eval(line.split(" ")[-4])
            kalman_position_y = eval(line.split(" ")[-2])
            all_measures_read = True
