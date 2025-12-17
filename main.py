"""
main

used with EPSON project
"""
import modbus_fxns
import camera_fxns

max_items = 33 # based on num spots in loc 100 on robot
total_good_spots = 15 # based on num spots in loc 100 on robot
total_bad_spots = 5

# start image window for non-blocking display
camera_fxns.start_img_window()

def end(client):
    """
    end the program by closing windows and resetting all bits
    """
    camera_fxns.cv2.destroyAllWindows()
    modbus_fxns.reset_bits(client, max_items)
    quit()

def main():
    """
    main

    takes initial photo for display, runs loop while pallets are not full
    
    """
    print("Initializing...")
    client = modbus_fxns.initialize_modbus('tcp')
    if modbus_fxns.check_robot_cycle_complete(client)==0:
        print('Robot cycle incomplete, exiting.')
        exit(1)
    modbus_fxns.reset_bits(client, max_items)
    H = camera_fxns.calculate_homography()
    modbus_fxns.time.sleep(1)
    # Take and preprocess photo
    orig_img = camera_fxns.take_photo()
    img, cropped, bad_img = camera_fxns.preprocess(orig_img) # preprocess for good items (white) AND bad ones (orange)
    img_coords = camera_fxns.find_items(img, cropped, True)
    img_coords_bad = camera_fxns.find_items(bad_img, cropped, False)
    # num_items = len(img_coords)
    camera_fxns.show_img(cropped)
    if camera_fxns.cv2.waitKey(1) & 0xFF == 27:  # ESC to exit
        end()
    modbus_fxns.time.sleep(1.5) # let img load
    ready_for_pickup = False
    total_items = 0
    total_good = 0
    total_bad = 0

    while total_items<(total_good_spots+total_bad_spots) and (total_good<total_good_spots) and (total_bad<total_bad_spots):
        # Start conveyor belt
        modbus_fxns.conveyor(client, 'on')

        # Take and preprocess photo
        orig_img = camera_fxns.take_photo()
        img, cropped, bad_img = camera_fxns.preprocess(orig_img) # preprocess for good items (white) AND bad ones (orange)
        img_coords = camera_fxns.find_items(img, cropped, True)
        img_coords_bad = camera_fxns.find_items(bad_img, cropped, False)
        # num_items = len(img_coords)
        camera_fxns.show_img(cropped)
        if camera_fxns.cv2.waitKey(1) & 0xFF == 27:  # ESC to exit
                break
        # check if bottles in view
        ready_for_pickup = camera_fxns.wait_for_items(img_coords, img_coords_bad)

        if ready_for_pickup:
            print("Detected items in ready area!")
            modbus_fxns.conveyor(client, 'off')
            modbus_fxns.time.sleep(0.5) # let conv turn off
            ready_for_pickup = False
            world_coords = []
            to_robot_coords = []
            world_coords_bad = []
            to_robot_coords_bad = []

            # Update photo and locations
            orig_img = camera_fxns.take_photo()
            img, cropped, bad_img = camera_fxns.preprocess(orig_img)
            img_coords = camera_fxns.find_items(img, cropped, True)
            num_items = len(img_coords)
            img_coords_bad = camera_fxns.find_items(bad_img, cropped, False)
            num_items_bad = len(img_coords_bad)

            camera_fxns.show_img(cropped)
            if camera_fxns.cv2.waitKey(1) & 0xFF == 27:  # ESC to exit
                break

            for img_coord in img_coords:
                world_x, world_y = camera_fxns.convert_pix_to_world(img_coord[0], img_coord[1], H)
                world_coords.append([world_x,world_y])
            print("GOOD items:")
            print(world_coords)

            for img_coord in img_coords_bad:
                world_x, world_y = camera_fxns.convert_pix_to_world(img_coord[0], img_coord[1], H, False)
                world_coords_bad.append([world_x,world_y])
            print("BAD items:")
            print(world_coords_bad)

            top_y_bound = 185
            bottom_y_bound = 30
            left_bound = 350
            right_bound = 750

            # Bounds check values just in case... possibly unnecessary tho bc of cropping
            for i in range(num_items):
                if(img_coords[i][1]<top_y_bound and img_coords[i][1]>bottom_y_bound and img_coords[i][0]>left_bound and img_coords[i][0]<right_bound): # check x and y bounds
                    to_robot_coords.append(world_coords[i])
            num_reachable = len(to_robot_coords)
            total_good+=num_reachable
            print(f"There are {num_items-num_reachable} GOOD items Scaramouche can't reach.")
            
            for i in range(num_items_bad):
                if(img_coords_bad[i][1]<top_y_bound and img_coords_bad[i][1]>bottom_y_bound and img_coords_bad[i][0]>left_bound and img_coords_bad[i][0]<right_bound): # check x and y bounds
                    to_robot_coords_bad.append(world_coords_bad[i])
            num_reachable_bad = len(to_robot_coords_bad)
            total_bad+=num_reachable_bad
            print(f"There are {num_items-num_reachable} BAD items Scaramouche can't reach.")
            
            # Send robot counts and loc values
            modbus_fxns.send_target_count(client, num_reachable)
            for i in range(num_reachable):
                modbus_fxns.send_modbus_coords(client,i+1,to_robot_coords[i][0],to_robot_coords[i][1])
            
            modbus_fxns.send_target_count(client, num_reachable_bad,False)
            for i in range(num_reachable_bad):
                modbus_fxns.send_modbus_coords(client,i+1,to_robot_coords_bad[i][0],to_robot_coords_bad[i][1], False)
                
            print(f"Scaramouche will now palletize {num_reachable} good items and {num_reachable_bad} bad ones.")

            modbus_fxns.set_modbus_bit(client, modbus_fxns.START_COMMAND, 1)
            modbus_fxns.time.sleep(1)
            modbus_fxns.set_modbus_bit(client, modbus_fxns.START_COMMAND, 0)
            # wait for robot to be ready again
            print("Waiting for robot to be done...")
            modbus_fxns.time.sleep(2)
            while modbus_fxns.check_robot_cycle_complete(client)==0:
                # print('Not ready yet...')
                modbus_fxns.time.sleep(1)
            print("done with cycle!")
            modbus_fxns.reset_bits(client, max_items)

            total_items=total_good+total_bad

    end(client)
   
main()