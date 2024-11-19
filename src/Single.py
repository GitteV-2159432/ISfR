from multiprocessing import Process



from main_FastSlam import main as FS
from main_lidar import main as L
from main_Map import main as M

import keyboard

if __name__ == '__main__':
    while 1 :
        

        p1 = Process(target=FS)
        p2 = Process(target=L)
        p3 = Process(target=M)

        p1.start()
        p2.start()  # A multiprocessing.Pool might be more efficient
        p3.start()

        p1.join()
        p2.join()   # Wait for all work to complete in both processes
        p3.join()

        # p1.close()
        # p2.close()
