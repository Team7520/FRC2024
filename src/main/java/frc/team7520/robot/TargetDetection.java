// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class TargetDetection {

    private PhotonCamera camera;
    private final PipeLineType type;
    private boolean IsOpen = false;
    private String camera_name;

    public enum PipeLineType {
        APRIL_TAG,
        COLORED_SHAPE,
        REFLECTIVE
    }

    public TargetDetection(String camera_name, PipeLineType type) {
        this.type = type;
        this.camera_name = camera_name;
        camera = new PhotonCamera(camera_name);
        if(camera.isConnected()) {
            IsOpen = true;
        } else {
            System.out.printf("Open Camera %s Fail !!!\n", camera_name);
        }
    }

    public class PhotonVisonData {
        public boolean is_vaild;
        // 2D Mode
        public double yaw;
        public double pitch;
        public double area;
        public double skew;
        // 3D Mode
        public int april_tag_id;
        public double x_distance;
        public double y_distance;
        public double z_distance;
        public double x_rotate;
        public double y_rotate;
        public double z_rotate;
        public double angle_rotate;
        public double ambiguity;
    }

    // Get PhotonVison Target Data
    public PhotonVisonData GetPVTargetData() {
        PhotonVisonData target_data = new PhotonVisonData();
        target_data.is_vaild = false;
        if(!IsOpen) {
            System.out.printf("Check PhotonVison Camera %s, which is NOT open", camera_name);
            camera = new PhotonCamera(camera_name);
            if(camera.isConnected()) {
                IsOpen = true;
                System.out.printf("PhotonVison Camera %s, opened now", camera_name);
            } else {    
                return target_data;
            }
        }

        var result = camera.getLatestResult();
        if(result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();
            
            // Get information from target.
            // The yaw of the target in degrees (positive right).            
            target_data.yaw = target.getYaw();
            
            // The pitch of the target in degrees (positive up).
            target_data.pitch = target.getPitch();
            
            // The area (how much of the camera feed the bounding box takes up) as a percent (0-100).            
            target_data.area = target.getArea();

            if(type == PipeLineType.APRIL_TAG) {
                
                // The ID of the detected fiducial marker.
                target_data.april_tag_id = target.getFiducialId();
                
                // How ambiguous the pose of the target is must less than 0.2 
                target_data.ambiguity = target.getPoseAmbiguity();

                /* Get the transform that maps camera space 
                   (X = forward, Y = left, Z = up) to object/fiducial tag space 
                   (X forward, Y left, Z up) with the lowest reprojection error.
                */
                Transform3d bestCameraToTarget = target.getBestCameraToTarget();
                
                target_data.x_distance = bestCameraToTarget.getX();
                target_data.y_distance = bestCameraToTarget.getY();
                target_data.z_distance = bestCameraToTarget.getZ();

                Rotation3d trans_3d_rotate = bestCameraToTarget.getRotation();
                target_data.x_rotate = trans_3d_rotate.getX();
                target_data.y_rotate = trans_3d_rotate.getY();
                target_data.z_rotate = trans_3d_rotate.getZ();
                target_data.angle_rotate = trans_3d_rotate.getAngle();

                // alternate Target, normally don't use it.
                //Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();
                target_data.is_vaild = true;
                System.out.printf("AprilTag Raw Data: yaw:%f,pitch:%f,area:%f, 3D: X:%f,Y:%f,Z:%f, RX:%f,RY:%f,RZ:%f,RW:%f, ID:%d, ambiguity:%f\n",
                            target_data.yaw, 
                            target_data.pitch, 
                            target_data.area, 
                            target_data.x_distance,
                            target_data.y_distance,
                            target_data.z_distance,
                            target_data.x_rotate,
                            target_data.y_rotate,
                            target_data.z_rotate,
                            target_data.angle_rotate,
                            target_data.april_tag_id, 
                            target_data.ambiguity);
            } else if(type == PipeLineType.COLORED_SHAPE) {               
                // The skew of the target in degrees (counter-clockwise positive).
                target_data.skew = target.getSkew(); 
                target_data.is_vaild = true;
                System.out.printf("ColoredShape Raw Data: yaw:%f,pitch:%f,area:%f,skew:%f\n",
                            target_data.yaw, 
                            target_data.pitch, 
                            target_data.area, 
                            target_data.skew);
            } else {
                System.out.println("Currently NOT support");
            }
        } else {
            System.out.println("Target is NOT found");
        }

        return target_data;
    }

}
