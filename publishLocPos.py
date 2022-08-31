from imu_pose_estimator import imu_data_source,pose_tracker,integrator
import ipywidgets as widgets

dataSource = imu_data_source()
estimator = pose_tracker(dataSource)#,ys,ps,rs)
loc_tracker = integrator(estimator,socketNum = 1337)

dataSource.start()

