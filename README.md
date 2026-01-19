# mocap_pose_to_odom


# Description 
ROS2 package that converts mocap data from Vicon to odometry messages 


# Troubleshooting 
In case of following error

```    
[ERROR] [] [mocap4r2_vicon_driver_node]: ... not connected :(  [ClientConnectionFailed]
```

Check: 
- Both computers (vicon and personal) are connected on the same network 
- Check vicon software permissions in Windows Defender (full permissions no restrictions)