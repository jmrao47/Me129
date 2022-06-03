import time
import threading
import traceback

from mapping_bot import MappingBot
import cfg 

def user_input(robot):
    while True:
        command = input('Command: ')
        command_args = command.split()

        if not robot.is_valid_task(command_args[0]):
            print('Invalid command!')
            continue 

        robot.stop_current_task()
        robot.start_task(*command_args)

if __name__ == "__main__":
    # Initialize global variables
    cfg.init()

    # Instantiate the low-level object
    robot = MappingBot()

    ultrasound_thread = threading.Thread(target = robot.ultrasound.run_continual)
    ultrasound_thread.start()

    driving_thread = threading.Thread(target = robot.driving_loop)
    driving_thread.start()

    # Run the code
    try:
        user_input(robot)

    except BaseException as ex:
        print("Ending due to exception: %s" % repr(ex))
        traceback.print_exc()

    # Shutdown cleanly
    robot.shutdown()

    # Wait for the triggering thread to be done
    robot.ultrasound.stop_continual()
    ultrasound_thread.join()
    driving_thread.join()