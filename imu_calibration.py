import qwiic_icm20948
import time
import sys
import numpy as np
import ipywidgets as widgets
from threading import Thread
from ellipsoid_fit_python.ellipsoid_fit import ellipsoid_fit, ellipsoid_plot, data_regularize

class calibrator:
    def __init__(self,center,transformation,radius=1):
        if isinstance(center,str) and isinstance(transformation,str):
            try:
                self.__init__(np.fromfile(center),np.fromfile(transformation).reshape(3,3),radius)
            except FileNotFoundError:
                self.center = np.array([0,0,0])
                self.transformation = np.identity(3)
        elif isinstance(center,np.ndarray) and isinstance(transformation,np.ndarray):
            self.center = center.T
            self.transformation = transformation/radius
        else:
            raise Exception('Bad input data types', 'use numpy arrays or strings') 
        
    def correct(self,data):
        data = data-self.center
        return self.transformation.dot(data.T).T


class dataCollector(Thread):
    
    
    def __init__(self,accelFile = None,gyroFile = None,magFile = None):
        super().__init__()
        self.paused = False
        self.collect = True
        self.Ascale = 65534/4.0
        self.Gscale = 65534/500.0
        self.Mscale = 10.0
        accel = np.array([])
        gyro = np.array([])
        mag = np.array([])
        if not accelFile==None:
            accel = np.fromfile(accelFile).reshape(-1,3)
        if not gyroFile==None:
            gyro = np.fromfile(gyroFile).reshape(-1,3)
        if not magFile==None:
            mag = np.fromfile(magFile).reshape(-1,3)
        self.accelData = accel.tolist()
        self.gyroData = gyro.tolist()
        self.magData = mag.tolist()
        self.IMU = qwiic_icm20948.QwiicIcm20948()

        if self.IMU.connected == False:
            print("The Qwiic ICM20948 device isn't connected to the system. Please check your connection", \
                file=sys.stderr)

        
    def run(self):
        self.IMU.begin()
        self.collect = True
        while self.collect:
            if self.IMU.dataReady():
                self.IMU.getAgmt() # read all axis and temp from sensor, note this also updates all instance variables
                if not self.paused:
                    self.accelData.append(np.array([self.IMU.axRaw/self.Ascale,\
                                                    self.IMU.ayRaw/self.Ascale,self.IMU.azRaw/self.Ascale]))
                    self.gyroData.append(np.array([self.IMU.gxRaw/self.Gscale,\
                                                   self.IMU.gyRaw/self.Gscale,self.IMU.gzRaw/self.Gscale]))
                    self.magData.append(np.array([self.IMU.mxRaw/self.Mscale,\
                                                  self.IMU.myRaw/self.Mscale,self.IMU.mzRaw/self.Mscale]))

                time.sleep(0.03)
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
        
    def getRawData(self):
        return np.array(self.accelData),np.array(self.gyroData),np.array(self.magData)
    
    def getCalibration(self):
        
        regAccelData = data_regularize(np.array(self.accelData))
        regMagData = data_regularize(np.array(self.magData))
        
        accelCenter, accelTR,accelRadius = ellipsoid_fit(regAccelData)
        magCenter, magTR,magRadius = ellipsoid_fit(regMagData)
        
        return calibrator(accelCenter,accelTR,accelRadius) , calibrator(magCenter,magTR,magRadius)
    
    def getGyroCal(self):
        gyroSum = np.sum(np.array(self.gyroData),axis=0)
        gyroAvg = gyroSum/len(self.gyroData)
        return calibrator(gyroAvg,np.identity(3))
        
