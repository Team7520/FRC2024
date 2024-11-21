package frc.team7520.robot.util;

import java.util.Arrays;
import java.util.List;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.ConstraintsZone;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.RotationTarget;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team7520.robot.constants.AutoConstants;
import frc.team7520.robot.subsystems.swerve.SwerveSubsystem;

public class PathPlannerHelper {
    static Pose2d lastPhotonPose2d;
    static Pose2d lastEndPose2d;
    static Pose2d lastMidPose2d;

    public static void initializeAutoBuilder(SwerveSubsystem s_Swerve){
        AutoBuilder.configureHolonomic(
            ()->s_Swerve.getPose(),
            (pose) -> {s_Swerve.resetOdometry(pose);},
            ()->s_Swerve.getRobotVelocity(),
            (chassisSpeeds) -> {s_Swerve.setChassisSpeeds(chassisSpeeds);},
            AutoConstants.config,
            getAllianceColorBooleanSupplier(),
            s_Swerve
        );
    }

    public static Command goToPoseCommand_preplanned(SwerveSubsystem s_Swerve, String pathName)
    {
        //s_Swerve.resetOdometry(s_Swerve.getPose());
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
        //path.preventFlipping = true;
        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.followPath(path);
    }

    
    public static Command goToPose(SwerveSubsystem s_Swerve, Pose2d endPose)
    {
        if (endPose == null) return null;

        // go to the endPose directly
        double endAngle=0;
        //endAngle = SmartDashboard.getNumber("endAngle", 0);
        Pose2d startPose = s_Swerve.getPose();
        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
            startPose,
            endPose
        );

        // Create the path using the bezier points created above
        PathPlannerPath path = new PathPlannerPath(
                bezierPoints,
                new PathConstraints(0.4, 1, Math.PI,  Math.PI), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
                new GoalEndState(0.0, Rotation2d.fromDegrees(endAngle)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        );

        // Prevent the path from being flipped if the coordinates are already correct
        path.preventFlipping =true;

        // Create a path following command using AutoBuilder. This will also trigger event markers.
        
        return AutoBuilder.followPath(path);
    }

    private static BooleanSupplier getAllianceColorBooleanSupplier(){
        return () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
        };
    }

    private static Pose2d ConvertToAbsolutePose(SwerveSubsystem s_Swerve, Pose2d srcPose)
    {
        if (srcPose == null) return null;

        var deltaPose = srcPose;
        Pose2d startPose = s_Swerve.getPose();

        Translation2d startTranslation2d = startPose.getTranslation();
        Translation2d deltaTranslation2d = deltaPose.getTranslation();
        Translation2d endTranslation2d = startTranslation2d.plus(deltaTranslation2d);
        Rotation2d starRotation2d = startPose.getRotation();
        Rotation2d deltaRotation2d = deltaPose.getRotation();
        Rotation2d endRotation2d = starRotation2d.plus(deltaRotation2d);

        Pose2d endPose = new Pose2d(endTranslation2d,endRotation2d);

        return endPose;
    }

    private static double getTotalDistance(List<Translation2d> waypoints) {
        double totalDistance = 0.0;

        for (int i = 1; i < waypoints.size(); i++) {
            Translation2d currentPoint = waypoints.get(i);
            Translation2d prevPoint = waypoints.get(i - 1);

            double distance = currentPoint.getDistance(prevPoint);
            totalDistance += distance;
        }

        return totalDistance;
    }
}
