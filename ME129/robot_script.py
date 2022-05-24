import time
import threading
import traceback

from mapping_bot import MappingBot

if __name__ == "__main__":
    # Instantiate the low-level object
    robot = MappingBot()
    thread = threading.Thread(target = robot.ultrasound.run_continual)
    thread.start()

    # Run the code
    try:
        #while True: 
        #    print(robot.ultrasound.get_distances())
        
        #time.sleep(1)
        #robot.drive_with_obstacles()

        #time.sleep(1)
        #print(robot.ultrasound.get_distances())
        #robot.wall_following()

        # input()

        # robot.ultrasound.trigger()
        # time.sleep(0.2)
        # print('distances:', robot.ultrasound.get_distances())
        # input()

        robot.map_course()
        lon = int(input('Target longitude: '))
        lat = int(input('Target latitude: '))

        robot.dijkstra((lon, lat))
        robot.drive_back()

    except BaseException as ex:
        print("Ending due to exception: %s" % repr(ex))
        traceback.print_exc()


    # Wait for the triggering thread to be done
    robot.ultrasound.stop_continual()
    thread.join()

    # Shutdown cleanly
    robot.shutdown()