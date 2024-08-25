package frc.team7520.robot.auto;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team7520.robot.subsystems.shooter.ShooterSubsystem;
import frc.team7520.robot.subsystems.swerve.SwerveSubsystem;


public class AutoTurn extends Command {
    private final SwerveSubsystem swerve;
    private final Rotation2d desiredheading;


    public AutoTurn(SwerveSubsystem swerve, Rotation2d desiredheading) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        this.swerve = swerve;
        this.desiredheading = desiredheading;
        addRequirements(swerve);
        

    }

    @Override
    public void initialize() {
        

    }

    @Override
    public void execute() {
        Rotation2d currentHeadingRotation2d = swerve.getHeading();
        Rotation2d desiredHeadingRotation2d = (new Rotation2d()).plus(desiredheading);
        double speedConstant = 0.06;
        Rotation2d difference = desiredHeadingRotation2d.minus(currentHeadingRotation2d);
        double turnSpeed = (difference.getDegrees())*speedConstant;
        swerve.drive(new Translation2d(0,0), turnSpeed, true);

        // double currentHeadingDeg = swerve.getHeading().getDegrees();
        // double desiredHeadingDeg = desiredheading.getDegrees();
        // double turnDirection = 0;
        // double speedConstant = 0.2;
        // double turnSpeed = Math.abs(desiredHeadingDeg-currentHeadingDeg)*speedConstant;
        // if (currentHeadingDeg != desiredHeadingDeg){
        //     if (Math.abs(desiredHeadingDeg-currentHeadingDeg)<180){ 
        //         if(desiredHeadingDeg>currentHeadingDeg){
        //             turnDirection = 1;
        //         } else {
        //             turnDirection = -1;
        //         }
        //     } else {
        //         if(desiredHeadingDeg>currentHeadingDeg){
        //             turnDirection = -1;
        //         } else {
        //             turnDirection = 1;
        //         }
        //     }
        // } else {
        //     turnSpeed = 0;
        // }
        // swerve.drive(swerve.getPose().getTranslation(), turnSpeed*turnDirection, true);
        
        // Using Pathplanner (on the fly path)
        // List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
        // );
        // PathPlannerPath turnpath = new PathPlannerPath(
        //     bezierPoints,
        //     new PathConstraints(1.0, 1.0, 2 * Math.PI, 2 * Math.PI),
        //     new GoalEndState(0.0, desiredheading) 
        // );
        // var cmd = AutoBuilder.followPath(turnpath);
        // cmd.schedule();

    }

    @Override
    public boolean isFinished() {
        return Math.abs((new Rotation2d()).plus(desiredheading).getDegrees()-swerve.getHeading().getDegrees())>3;
    }


    @Override
    public void end(boolean interrupted) {

    }
}
