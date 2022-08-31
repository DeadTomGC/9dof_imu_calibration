import qwiic_icm20948
import time
import sys
import numpy as np
from threading import Thread
from imu_calibration import calibrator
from fusion import Fusion
import pyquaternion as pq
import zmq

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
        self.Ascale = 65534/8.0
        self.Gscale = 65534/1000.0
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
        self.IMU.setFullScaleRangeAccel(qwiic_icm20948.gpm4)
        self.IMU.setFullScaleRangeGyro(qwiic_icm20948.dps500)
        #self.IMU.enableDlpfAccel(True)
        #self.IMU.enableDlpfGyro(True)
        #self.IMU.setDLPFcfgAccel(qwiic_icm20948.acc_d50bw4_n68bw8)
        #self.IMU.setDLPFcfgGyro(qwiic_icm20948.gyr_d119bw5_n154bw3)
        self.collect = True
        t0 = time.time()
        while self.collect:
            
            #if self.IMU.dataReady():
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
            #else:
                #print("Waiting for data")
                #time.sleep(0.5)
            
            
                
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
    
    def __init__(self,data_source,dtSlider = None,yawSlider = None,pitchSlider = None,rollSlider = None\
                 ,accelCal=calibrator('data/accelCenter','data/accelTrans'),\
                 gyroCal=calibrator('data/gyroCenter','data/gyroTrans'),magCal=calibrator('data/magCenter','data/magTrans')):
        self.data_source = data_source
        self.magCal = magCal
        self.gyroCal = gyroCal
        self.accelCal = accelCal
        self.orientation = np.array([1,0,0])
        self.lagCount = 0
        self.count = 0
        self.fus = Fusion()
        self.dtSlider = dtSlider
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
        #t0=time.time()
        accel,gyro,mag = self.calibrateData(accel,gyro,mag)
        #accel[0] = accel[0]-0.01
        #accel[1] = accel[1]+0.005
        #accel[2] = accel[2]-0.01
        oldq = self.fus.q
        
        
        
        self.fus.update(tuple(accel),tuple(gyro),tuple([mag[0],-mag[1],-mag[2]]),deltat)
        #deltat = time.time()-t0
        hpr = [self.fus.heading,self.fus.pitch,self.fus.roll]
        for ad in self.subscribers:
            ad(accel,gyro,mag,oldq,self.fus.q,deltat,hpr)
        
        
        
        self.count+=1
        
        if self.count > 50 and not self.yawSlider == None\
            and not self.pitchSlider == None\
            and not self.rollSlider == None:
            self.yawSlider.value = self.fus.heading
            self.pitchSlider.value = self.fus.pitch
            self.rollSlider.value = self.fus.roll
            self.count=0
            
        if deltat>0.016:
            self.lagCount+= 1
        if self.count > 50 and not self.dtSlider == None:
            self.dtSlider.value = self.lagCount
            self.count=0
            
class integrator:
    
    def __init__(self,pose_tracker,xSlider = None,ySlider = None,zSlider = None,socketNum = 1337):
        self.pose_tracker = pose_tracker
        self.velocity = np.array([0.0,0.0,0.0])
        self.location = np.array([0.0,0.0,0.0])
        self.gravity = np.array([0.0,0.0,1.0])
        self.accels = []
        self.accels_len = 0
        self.pose_tracker.subscribe(self.addData)
        self.xSlider = xSlider
        self.ySlider = ySlider
        self.zSlider = zSlider
        self.avgAccel = np.array([0.0,0.0,0.0])
        self.count = 0
        self.cycles = 0;
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind("tcp://*:%s" % str(socketNum))
        #self.accel_offset = np.array([0.0,0.0,0.0])
        #self.accel_avg = np.array([0.0,0.0,0.0])
        
    def addData(self,accel,gyro,mag,oldquat,quat,deltat, hpr = None):
        
        if hpr == None:
            hpr = [0,0,0]
        
        rotQuat = pq.Quaternion.slerp(pq.Quaternion(oldquat),pq.Quaternion(quat),0.5)
        
        RM = quaternion_rotation_matrix(quat)
        accel_in_real_world = RM.dot(accel.T)
        accel_nograv = accel_in_real_world - self.gravity
        
        
        self.cycles+=1
        #if self.cycles <= 1500:
        #    self.accel_avg += accel
        #if self.cycles == 1500:
        #    self.accel_avg = self.accel_avg/1500
        #    self.accel_offset = self.accel_avg/np.linalg.norm(self.accel_avg) - self.accel_avg
        if self.cycles > 1500:
            
            
            
                
            #if self.accels_len >= 1000:
                #self.avgAccel = sum(self.accels)/self.accels_len
                #self.accels.pop(0)
                #self.accels_len-=1
            #self.accels.append(9.8*accel_nograv)
            #self.accels_len+=1
            if self.cycles > 2000:
                self.velocity = self.velocity + 9.8*accel_nograv*deltat
                self.location = self.location + self.velocity*deltat
                if np.linalg.norm(self.location)>1:
                    self.velocity = np.array([0,0,0])
                    self.location = np.array([0,0,0])
                
                #if np.linalg.norm(self.velocity)>=0.001: #random step compensation (possible bad idea)
                    #self.velocity = self.velocity - 0.0005*self.velocity/np.linalg.norm(self.velocity)
        self.socket.send_string("%f %f %f %f %f %f %f" % (self.location[0], self.location[1], self.location[2], hpr[0],hpr[1],hpr[2],deltat))
        self.count+=1
        
        if self.count > 20 and not self.xSlider == None\
            and not self.ySlider == None\
            and not self.zSlider == None:
            self.xSlider.value = self.velocity[0]
            self.ySlider.value = self.velocity[1]
            self.zSlider.value = self.velocity[2]
            self.count=0
        
        
        
        #print((self.location[0], self.location[1], self.location[2], hpr[0],hpr[1],hpr[2]))

    def resetData(self,sender):
        self.velocity = np.array([0,0,0])
        self.location = np.array([0,0,0])