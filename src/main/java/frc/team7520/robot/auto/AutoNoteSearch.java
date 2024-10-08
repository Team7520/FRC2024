package frc.team7520.robot.auto;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team7520.robot.Constants;
import frc.team7520.robot.commands.Shooter;
import frc.team7520.robot.subsystems.intake.IntakeSubsystem;
import frc.team7520.robot.subsystems.shooter.ShooterSubsystem;
import frc.team7520.robot.subsystems.swerve.SwerveSubsystem;


public class AutoNoteSearch extends Command {
    private final SwerveSubsystem swerve;
    private final IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();
    private final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
    private double currentHeading;


    public AutoNoteSearch(SwerveSubsystem swerve) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        this.swerve = swerve;
        addRequirements(swerve);
        addRequirements(intakeSubsystem);
        addRequirements(shooterSubsystem);
        

    }

    @Override
    public void initialize() {
        intakeSubsystem.setPosition(Constants.IntakeConstants.Position.SHOOT);
        intakeSubsystem.setSpeed(0);
        currentHeading = 0;
        shooterSubsystem.setSpeed(0, false);

    }

    @Override
    public void execute() {
        currentHeading++;
        if (SwerveSubsystem.isBlueAlliance) {
            swerve.drive(new Translation2d(0,0), Math.PI/3, true);
        } else {
            swerve.drive(new Translation2d(0,0), -Math.PI/3, true);
        }
        

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
        return swerve.noteAvailable || currentHeading > 300;
    }


    @Override
    public void end(boolean interrupted) {

    }
}
