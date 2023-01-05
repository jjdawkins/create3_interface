# !/bin/python3
import rclpy
import time
import math
import threading
from create3_ros import IRobotCreate

rclpy.init()
# Establish the connection to the Create as a IRobotCreate object
my_create = IRobotCreate('')

def run_loop():
    """
    run_loop function runs an audio sequence as long
    as rclpy is at status ok.
    """
    
    # Get current clock time off of Create and set it as initial time
    init_time = my_create.get_clock().now()
    # while loop continues to run as long as ros2 client library is in ok status
    while rclpy.ok:
        
        # duration since initial clock time (i.e. elapsed time since start)
        dur = my_create.get_clock().now() - init_time
        # convert nanosecond elapsed time into second
        t = (dur.nanoseconds)*(1e-9)

        # Play C scale one note at a time
        my_create.set_audio(262,1) # frequency 262 Hz, duration 1 second
        time.sleep(2) # sleep 2 seconds to leave 1 second pause between notes
        my_create.set_audio(294,1)
        time.sleep(2)
        my_create.set_audio(330,1)
        time.sleep(2)
        my_create.set_audio(350,1)
        time.sleep(2)
        my_create.set_audio(392,1)
        time.sleep(2)
        my_create.set_audio(440,1)
        time.sleep(2)
        my_create.set_audio(494,1)
        time.sleep(2)
        my_create.set_audio(523,1)
        time.sleep(4) # wait 4 seconds before starting sequence again



def main(args=None):    
    """ 
    Main function to run the audio demo script.
    """
    
    # Establish a thread to run the run_loop function in parallel with other threads as needed
    thrd = threading.Thread(target=run_loop)

    # start the run_loop thread
    thrd.start()
    
    # spin the my_create object through one cycle
    rclpy.spin(my_create)
    # block the my_create processes until the run_loop thread has terminated
    thrd.join()
    # Delete/destroy the my_create object to disable interaction with vehicle
    my_create.destroy_node()
    # shut down the ros2 client library running under teh hood
    rclpy.shutdown()


if __name__ == '__main__':
    main()