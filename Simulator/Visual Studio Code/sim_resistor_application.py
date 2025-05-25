import sim_scara_motion_controller_api as ssrc_api
import time
import random

scara = ssrc_api.sim_scara_motion_controller(host='127.0.0.1', port=9000)

tool_offset = 0.067
# picking tray scanning coordinates
starting_x = [0.000, -0.020, -0.020, 0.000, 0.020, 0.020, 0.020, 0.000, -0.020]
starting_y = [0.130,  0.130,  0.150, 0.150, 0.150, 0.130, 0.110, 0.110,  0.110]
starting_z = 0.022 + tool_offset
picking_z = -0.036 + tool_offset
# location and orientation of testing point [x, y, z_angle, z]
testing_z = -0.036 + tool_offset
testing_point_coord = [0.0925, 0.1575, 90.0, testing_z]
# location and orientation of sorting trays [x, y, z_angle, z]
sorting_y_10 = 0.1175
sorting_y_50 = 0.0925
sorting_y_100 = 0.0675
sorting_y_500 = 0.0425
sorting_y_1000 = 0.0125
sorting_y_5000 = -0.0125
sorting_y_10000 = -0.0425
sorting_y_other = -0.0675
sorting_y = [sorting_y_10, sorting_y_50, sorting_y_100, sorting_y_500,
             sorting_y_1000, sorting_y_5000, sorting_y_10000, sorting_y_other]
sorting_point_coord = [0.130, sorting_y[7], 90.0, picking_z] # default to 'other'

while True:    
    for i in range(9):

        # ==================== PICKING ====================
        # move arm above picking tray
        scara.SCARA_MOVE_COORD(starting_x[i], starting_y[i], 0.0, starting_z, 0.5)
        # open gripper
        # <- ->

        scara.SCARA_MOVE_COORD
        scara.SCARA_MOVE_COORD(starting_x[i], starting_y[i], 0.0, starting_z, 0.5)
        # move arm down
        scara.SCARA_MOVE_COORD(starting_x[i], starting_y[i], 0.0, picking_z, 1.0)
        time.sleep(0.5)
        # close gripper
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
        time.sleep(0.5)
        # read resistor value        
        # move arm up
        scara.SCARA_MOVE_COORD(testing_point_coord[0], testing_point_coord[1], testing_point_coord[2], starting_z, 0.5)

        # ==================== SORTING ====================
        # simulate unknown resistor value
        n = random.randint(0, 7)
        print(f"random tray: {n}")
        # go to dispose tray
        scara.SCARA_MOVE_COORD(sorting_point_coord[0], sorting_y[n], 0.0, starting_z, 0.5)
        # move arm down
        scara.SCARA_MOVE_COORD(sorting_point_coord[0], sorting_y[n], sorting_point_coord[2], sorting_point_coord[3], 0.5)
        time.sleep(0.5)
        # open gripper
        # <- ->
        time.sleep(0.5)
        # move arm up
        scara.SCARA_MOVE_COORD(sorting_point_coord[0], sorting_y[n], 0.0, starting_z, 0.5)
        # gripper idle
        # -| |-
        # move back to picking tray
        scara.SCARA_MOVE_COORD(starting_x[i], starting_y[i], 0.0, starting_z, 0.5)

    again = int(input("again? "))
    if again == 0:
        break