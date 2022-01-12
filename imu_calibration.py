import qwiic_icm20948
import time
import sys
import numpy as np
import ipywidgets as widgets
from threading import Thread
from ellipsoid_fit import ellipsoid_fit, ellipsoid_plot, data_regularize

class calibrator:
    def __init__(self,center,transformation,radius):
        self.center = center.T
        self.transformation = transformation/radius
        
    def correct(self,data):
        data = data-self.center
        return self.transformation.dot(data.T).T


class dataCollector(Thread):
    
    
    def __init__(self):
        super().__init__()
        self.paused = False
        self.collect = True
        self.Ascale = 65534/4.0
        self.Gscale = 65534/500.0
        self.Mscale = 10.0
        self.accelData = []
        self.gyroData = []
        self.magData = []
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
                #    print(\
                # 'Accel X:{: 06f}g'.format(IMU.axRaw/Ascale)\
                #, '\n', 'Accel Y:{: 06f}g'.format(IMU.ayRaw/Ascale)\
                #, '\n', 'Accel Z:{: 06f}g'.format(IMU.azRaw/Ascale)\
                #, '\n', 'Gyro X{: 06f}dps'.format(IMU.gxRaw/Gscale)\
                #, '\n', 'Gyro Y{: 06f}dps'.format(IMU.gyRaw/Gscale)\
                #, '\n', 'Gyro Z{: 06f}dps'.format(IMU.gzRaw/Gscale)\
                #, '\n', 'Mag X{: 06f}'.format(IMU.mxRaw/Mscale)\
                #, '\n', 'Mag Y{: 06f}'.format(IMU.myRaw/Mscale)\
                #, '\n', 'Mag Z{: 06f}'.format(IMU.mzRaw/Mscale)\
                #)
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
        
