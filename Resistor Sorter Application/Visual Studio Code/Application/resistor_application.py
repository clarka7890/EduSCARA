import scara_motion_controller_api as smc_api
import servo_gripper_ohmmeter_api as sgo_api
import gripper_camera_api as gca_api

import serial.tools.list_ports
import time
import cv2

ports = list(serial.tools.list_ports.comports())
if not ports:
    print("No USB device found.")
else:
    port_list = [f"{port.device} - {port.description}" for port in ports]
    print(f"Available Ports:\n" + "\n".join(port_list))

    scara_port = input("\nEnter the COM port for SCARA (e.g., COM3):")
    scara = smc_api.scara_motion_controller(port=scara_port)

    gripper_port = input("\nEnter the COM port for Gripper (e.g., COM3):")
    gripper = sgo_api.servo_gripper_ohmmeter(port=gripper_port)

    resistor_detector = gca_api.resistor_detector(screen_width_m=0.035, screen_height_m=0.02)

    gripper.IDLE()

    link_1 = 0.125
    link_2 = 0.1
    z_min = 0.095
    z_max = 0.15
    settling_time = 0.5
    P_0 = 1.3
    I_0 = 0.01
    D_0 = 0.001
    P_1 = 1.3
    I_1 = 0.01
    D_1 = 0.001
    
    scara.SCARA_INITIALISE(link_1, link_2, z_min, z_max,
                            settling_time,
                            P_0, I_0, D_0, P_1, I_1, D_1)
    scara.SCARA_MOVE_JOINT(2, 0.0, 1.0)
    scara.SCARA_MOVE_JOINT(3, -60, 1.0)
    scara.SCARA_MOVE_JOINT(0, 0.0, 1.0)
    scara.SCARA_MOVE_JOINT(1, 90.0, 1.0)
    scara.SCARA_AUTO_CALIBRATE()

    # picking tray scanning coordinates
    starting_x = [0.000, -0.020, -0.020, 0.000, 0.020, 0.020, 0.020, 0.000, -0.020]
    starting_y = [0.138,  0.138,  0.158, 0.158, 0.158, 0.138, 0.118, 0.118,  0.118]
    starting_z = 0.15
    picking_z = 0.107
    # location and orientation of testing point [x, y, z_angle, z]
    testing_z = 0.109
    testing_point_coord = [0.092, 0.157, 90.0, testing_z]
    # location and orientation of sorting trays [x, y, z_angle, z]
    sorting_y_10 = 0.12
    sorting_y_50 = 0.095
    sorting_y_100 = 0.07
    sorting_y_500 = 0.045
    sorting_y_1000 = 0.015
    sorting_y_5000 = -0.007
    sorting_y_10000 = -0.032
    sorting_y_other = -0.06
    sorting_y = [sorting_y_10, sorting_y_50, sorting_y_100, sorting_y_500,
                sorting_y_1000, sorting_y_5000, sorting_y_10000, sorting_y_other]
    sorting_point_coord = [0.135, sorting_y[7], 90.0, picking_z] # default to 'other'
    
    while True:    
        for i in range(9):
            # ==================== PICKING ====================
            # move arm above picking tray
            scara.SCARA_MOVE_COORD(starting_x[i], starting_y[i], 0.0, starting_z, 0.5)
            # open gripper
            gripper.OPEN()
            # <- ->
            
            # --------------- LOOK FOR RESISTOR ---------------
            # repeatedly move to center the resistor
            max_attempts = 10
            attempts = 0
            current_x, current_y, _, _ = scara.SCARA_READ_COORD()

            while attempts < max_attempts:
                pos = resistor_detector.find_resistors(draw=True)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    cv2.destroyAllWindows()
                    break

                if pos:
                    dx, dy, zangle = pos
                    distance = (dx**2 + dy**2)**0.5
                    print(f"[attempt {attempts+1}] resistor offset: dx={dx:.5f} m, dy={dy:.5f} m, dist={distance:.5f} m")

                    if distance < 0.003:
                        print("resistor is centered!")
                        break
                    P = 0.5
                    # move towards the resistor
                    current_x, current_y, _, _ = scara.SCARA_READ_COORD()
                    current_x += dx * P
                    current_y += dy * P

                    scara.SCARA_MOVE_COORD(current_x, current_y, 0.0, 0.15, 0.1)
                    time.sleep(0.5)

                else:
                    print("no resistor detected.")
                
                attempts += 1
            # -------------------------------------------------

            # if resistor detected
            if pos:
                # move arm down
                scara.SCARA_MOVE_COORD(current_x, current_y, zangle, picking_z, 0.5)
                # close gripper
                gripper.CLOSE()
                # -> <-
                time.sleep(0.5)
                # move arm up
                scara.SCARA_MOVE_COORD(starting_x[i], starting_y[i], 0, starting_z, 0.5)
                
                # ==================== TESTING ====================
                # move arm above tester
                scara.SCARA_MOVE_COORD(testing_point_coord[0], testing_point_coord[1], testing_point_coord[2], starting_z, 0.5)
                # move arm down
                scara.SCARA_MOVE_COORD(testing_point_coord[0], testing_point_coord[1], testing_point_coord[2], testing_point_coord[3], 0.5)
                # wait for resistor test result
                time.sleep(1.0)
                # read resistor value
                resistance = float(gripper.TEST())        
                # move arm up
                scara.SCARA_MOVE_COORD(testing_point_coord[0], testing_point_coord[1], testing_point_coord[2], starting_z, 0.5)

                # ==================== SORTING ====================
                # sort based on resistor value
                if resistance >= 10000:
                    n = 7
                elif resistance < 10000 and resistance >= 5000:
                    n = 6
                elif resistance < 5000 and resistance >= 1000:
                    n = 5
                elif resistance < 1000 and resistance >= 500:
                    n = 4
                elif resistance < 500 and resistance >= 100:
                    n = 3
                elif resistance < 100 and resistance >= 50:
                    n = 2
                elif resistance < 50 and resistance >= 10:
                    n = 1
                else:  # resistance < 10
                    n = 0
                # go to dispose tray
                scara.SCARA_MOVE_COORD(sorting_point_coord[0], sorting_y[n], 0.0, starting_z, 0.5)
                # move arm down
                scara.SCARA_MOVE_COORD(sorting_point_coord[0], sorting_y[n], sorting_point_coord[2], sorting_point_coord[3], 0.5)
                time.sleep(0.5)
                # open gripper
                gripper.OPEN()
                # <- ->
                time.sleep(0.5)
                # close gripper
                gripper.CLOSE()
                # -> <-
                # move arm up
                scara.SCARA_MOVE_COORD(sorting_point_coord[0], sorting_y[n], 0.0, starting_z, 0.5)
                # gripper idle
                gripper.IDLE()
                # -| |-
                # move back to picking tray
                scara.SCARA_MOVE_COORD(starting_x[i], starting_y[i], 0.0, starting_z, 0.5)
            else:
                print("resistor not found at position")

        again = int(input("again? "))
        if again == 0:
            break