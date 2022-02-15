import qwiic_icm20948
import time
import sys
import numpy as np
from threading import Thread
from imu_calibration import calibrator
from fusion import Fusion

def quaternion_rotation_matrix(Q):
    """
    Covert a quaternion into a full three-dimensional rotation matrix.
 
    Input
    :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 
 
    Output
    :return: A 3x3 element matrix representing the full 3D rotation matrix. 
             This rotation matrix converts a point in the local reference 
             frame to a point in the global reference frame.
    """
    # Extract the values from Q
    q0 = Q[0]
    q1 = Q[1]
    q2 = Q[2]
    q3 = Q[3]
     
    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)
     
    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)
     
    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1
     
    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])
                            
    return rot_matrix

class imu_data_source(Thread):
    
    
    def __init__(self):
        super().__init__()
        self.paused = False
        self.collect = True
        self.Ascale = 65534/4.0
        self.Gscale = 65534/500.0
        self.Mscale = 10.0
        self.subscribers = []
        self.accelData = []
        self.gyroData = []
        self.magData = []
        self.times = []
        self.IMU = qwiic_icm20948.QwiicIcm20948()
        
        if self.IMU.connected == False:
            print("The Qwiic ICM20948 device isn't connected to the system. Please check your connection", \
                file=sys.stderr)

    
    def run(self):
        self.IMU.begin()
        self.collect = True
        t0 = time.time()
        while self.collect:
            
            if self.IMU.dataReady():
                self.IMU.getAgmt() # read all axis and temp from sensor, note this also updates all instance variables
                if not self.paused:   
                    
                    accel = np.array([self.IMU.axRaw/self.Ascale,\
                                                    self.IMU.ayRaw/self.Ascale,self.IMU.azRaw/self.Ascale])
                    gyro = np.array([self.IMU.gxRaw/self.Gscale,\
                                                   self.IMU.gyRaw/self.Gscale,self.IMU.gzRaw/self.Gscale])
                    mag = np.array([self.IMU.mxRaw/self.Mscale,\
                                                  self.IMU.myRaw/self.Mscale,self.IMU.mzRaw/self.Mscale])
                    deltat = time.time()-t0
                    t0=time.time()
                    for ad in self.subscribers:
                        ad(accel,gyro,mag,deltat)
                    
                    #self.times.append(deltat)
            
                    #if len(self.times)%30 == 0:
                        #print(self.times)

                #time.sleep(0.03)
            else:
                print("Waiting for data")
                time.sleep(0.5)
            
            
                
    def togglePauseData(self,b=None):
        self.paused = False if self.paused else True
        if b is not None:
            b.description = 'Resume' if self.paused else 'Pause'
            
    def stop(self,b=None):
        self.collect = False
        self.join()
        
    def subscribe(self,addData):
        self.subscribers.append(addData)
    
class pose_tracker:
    
    def __init__(self,data_source,yawSlider = None,pitchSlider = None,rollSlider = None\
                 ,accelCal=calibrator('data/accelCenter','data/accelTrans'),\
                 gyroCal=calibrator('data/gyroCenter','data/gyroTrans'),magCal=calibrator('data/magCenter','data/magTrans')):
        self.data_source = data_source
        self.magCal = magCal
        self.gyroCal = gyroCal
        self.accelCal = accelCal
        self.orientation = np.array([1,0,0])
        self.count = 0
        self.fus = Fusion()
        self.yawSlider = yawSlider
        self.pitchSlider = pitchSlider
        self.rollSlider = rollSlider
        self.data_source.subscribe(self.addData)
        self.subscribers = []
        
    def calibrateData(self,accel,gyro,mag):
        return self.accelCal.correct(accel),self.gyroCal.correct(gyro),self.magCal.correct(mag)
        
    def subscribe(self,addData):
        self.subscribers.append(addData)
        
    def addData(self,accel,gyro,mag,deltat):
        accel,gyro,mag = self.calibrateData(accel,gyro,mag)
        self.fus.update(tuple(accel),tuple(gyro),tuple([mag[0],-mag[1],-mag[2]]),deltat)
        
        for ad in self.subscribers:
            ad(accel,gyro,mag,self.fus.q,deltat)
        
        self.count+=1
        
        if self.count > 20 and not self.yawSlider == None\
            and not self.pitchSlider == None\
            and not self.rollSlider == None:
            self.yawSlider.value = self.fus.heading
            self.pitchSlider.value = self.fus.pitch
            self.rollSlider.value = self.fus.roll
            #print(self.fus.heading,self.fus.pitch,self.fus.roll)
            self.count=0
class integrator:
    
    def __init__(self,pose_tracker):
        self.pose_tracker = pose_tracker
        self.velocity = np.array([0,0,0])
        self.gravity = np.array([0,0,1])
        self.pose_tracker.subscribe(self.addData)
        self.count = 0
        self.cycles = 0;
        
    def addData(self,accel,gyro,mag,quat,deltat):
        RM = quaternion_rotation_matrix(quat)
        accel_in_real_world = RM.dot(accel.T)
        accel_nograv = accel_in_real_world-self.gravity
        self.velocity = self.velocity + 9.8*accel_nograv*deltat
        self.cycles+=1
        if self.cycles > 200:
            self.gravity = self.gravity*0.999 + accel_in_real_world*0.001
        if np.linalg.norm(self.velocity)>=0.01: #random step compensation (possible bad idea)
            self.velocity = self.velocity - 0.001*self.velocity/np.linalg.norm(self.velocity)
            
        self.count+=1
        
        if self.count > 100:
            print(self.velocity)
            #print(accel_in_real_world)
            #print(accel)
            #print(RM)
            self.count=0
