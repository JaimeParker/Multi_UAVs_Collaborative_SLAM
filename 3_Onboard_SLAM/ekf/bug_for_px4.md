# EFK2 fail to change to Vision and keeps reporting requesting home position

I'm facing a problem trying to apply SLAM on UAV in gazebo, so that I need to revise 2 parameters.

edit file: `PX4/ROMFS/px4fmu_common/init.d-posix/rcS`

* param set EKF2_AID_MASK 24
* param set EKF2_HGT_MODE 3

which means using Vision for localization and height measurement instead of GPS.

After that I deleted 2 files or folder, using command `rm ~/.ros/eeprom/parameters*` and `rm -rf ~/.ros/sitl*`, so that PX4 can reload parameters.

Finally I run command `roslaunch px4 mavros_posix_sitl.launch`.(I've already edited, add iris_stereo_camera in it)

But 

```
INFO  [px4] Startup script returned successfully
pxh> INFO  [commander] Failsafe mode activated	
pxh> commander takeoff
pxh> WARN  [commander] Takeoff denied! Please disarm and retry	
INFO  [tone_alarm] notify negative
INFO  [commander] Armed by internal command	
INFO  [tone_alarm] battery warning (fast)
pxh> commander disarm
pxh> INFO  [commander] Disarmed by internal command	
INFO  [logger] closed logfile, bytes written: 21143770
pxh> commander takeoff
pxh> WARN  [commander] Takeoff denied! Please disarm and retry	
INFO  [tone_alarm] notify negative
INFO  [commander] Armed by internal command	
INFO  [tone_alarm] battery warning (fast)
INFO  [logger] Start file log (type: full)
INFO  [logger] [logger] ./log/2022-03-17/07_00_55.ulg	
INFO  [logger] Opened full log file: ./log/2022-03-17/07_00_55.ulg
INFO  [commander] Disarmed by auto preflight disarming	
INFO  [logger] closed logfile, bytes written: 2061585
```

