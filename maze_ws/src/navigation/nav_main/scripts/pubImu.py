#!/usr/bin/env python3

import rospy

            if (self.useImu == True) :
                try:
                    stat_, yaw, yaw_vel, acc_x, acc_y, acc_z = self.Microcontroller.get_imu_val()
                    # Degree to radians
                    yaw = yaw * 3.1415926 / 180.0
                    isValid = True
                    if yaw == 0.0:
                        isValid = False

                    # print("yaw:  " + str(yaw)+" yaw_vel: " + str(yaw_vel)+" acc_x: " + str(acc_x)+" acc_y: " + str(acc_y)+" acc_z: " + str(acc_z))
                    if isValid:
                        # if yaw>=18000:
                        #     yaw = yaw-65535
                        # yaw = yaw/100.0
                        # if yaw_vel>=32768:
                        #     yaw_vel = yaw_vel-65535
                        # yaw_vel = yaw_vel/100.0
                        
                        imu_data = Imu()  
                        imu_data.header.stamp = rospy.Time.now()
                        imu_data.header.frame_id = self.imu_frame_id 
                        imu_data.orientation_covariance[0] = 1000000
                        imu_data.orientation_covariance[1] = 0
                        imu_data.orientation_covariance[2] = 0
                        imu_data.orientation_covariance[3] = 0
                        imu_data.orientation_covariance[4] = 1000000
                        imu_data.orientation_covariance[5] = 0
                        imu_data.orientation_covariance[6] = 0
                        imu_data.orientation_covariance[7] = 0
                        imu_data.orientation_covariance[8] = 0.000001

                        newquat = quaternion_from_euler(0, 0, yaw)
                        imu_data.orientation = Quaternion(newquat[0], newquat[1], newquat[2], newquat[3])
                        imu_data.linear_acceleration_covariance[0] = -1
                        imu_data.angular_velocity_covariance[0] = -1

                        imu_data.linear_acceleration.x = acc_x
                        imu_data.linear_acceleration.y = acc_y
                        imu_data.linear_acceleration.z = acc_z

                        imu_data.angular_velocity.x = 0.0
                        imu_data.angular_velocity.y = 0.0
                        imu_data.angular_velocity.z = yaw_vel
                        self.imuPub.publish(imu_data)
                        self.imuAnglePub.publish(yaw)
                except:
                    self.bad_encoder_count += 1
                    rospy.logerr("IMU exception count: " + str(self.bad_encoder_count))
                    return
     
            dt = now - self.then
            self.then = now
            dt = dt.to_sec()
            
            # Calculate odometry
            dright = (1.0 * right_enc) / self.ticks_per_meter
            dleft = (1.0 * left_enc) / self.ticks_per_meter
            # print(dright, dleft, right_enc, left_enc)

            dxy_ave = (dright + dleft) / 2.0
            dth = (dright - dleft) / self.wheel_track
            vxy = dxy_ave / dt
            vth = dth / dt
                
            if (dxy_ave != 0):
                dx = cos(dth) * dxy_ave
                dy = -sin(dth) * dxy_ave
                self.x += (cos(self.th) * dx - sin(self.th) * dy)
                self.y += (sin(self.th) * dx + cos(self.th) * dy)
    
            if (dth != 0):
                self.th += dth 
    
            quaternion = Quaternion()
            quaternion.x = 0.0 
            quaternion.y = 0.0
            quaternion.z = sin(self.th / 2.0)
            quaternion.w = cos(self.th / 2.0)
    
            # Create the odometry transform frame broadcaster.
            if (self.useImu == False) :
                self.odomBroadcaster.sendTransform(
                  (self.x, self.y, 0), 
                  (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                  rospy.Time.now(),
                  self.base_frame,
                  "odom"
                )
    
            odom = Odometry()
            odom.header.frame_id = "odom"
            odom.child_frame_id = self.base_frame
            odom.header.stamp = now
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0
            odom.pose.pose.orientation = quaternion
            odom.twist.twist.linear.x = vxy
            odom.twist.twist.linear.y = 0
            odom.twist.twist.angular.z = vth

            odom.pose.covariance = ODOM_POSE_COVARIANCE
            odom.twist.covariance = ODOM_TWIST_COVARIANCE

            self.odomPub.publish(odom)
            
            if now > (self.last_cmd_vel + rospy.Duration(self.timeout)):
                self.v_x = 0
                self.v_y = 0
                self.v_th = 0

            # Set motor speeds in encoder ticks per PID loop
            if ((not self.stopped)):
                self.Microcontroller.drive(self.v_x, self.v_th)
                
            self.t_next = now + self.t_delta