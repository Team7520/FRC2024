package frc.team7520.robot.auto;



import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team7520.robot.Constants.IntakeConstants.Position;
import frc.team7520.robot.subsystems.intake.IntakeSubsystem;
import frc.team7520.robot.subsystems.shooter.ShooterSubsystem;
import frc.team7520.robot.subsystems.swerve.SwerveSubsystem;


public class AutoTurn extends Command {
    private final SwerveSubsystem swerve;
    
    //private final IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();
    private Rotation2d desiredheading;
    int mode;
    private  double speedConstant = 2;
    private final static double BASE_SPEED = Math.PI/4;

    /** 
     * @param swerve the swerve Subsystem
     * @param desiredheading the desired direction in degrees
     */
    public AutoTurn(SwerveSubsystem swerve, int mode, Rotation2d desiredheading) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        this.swerve = swerve;
        this.mode = mode;
        this.desiredheading = desiredheading; 
        addRequirements(swerve);
        //addRequirements(intakeSubsystem);
        

    }

    @Override
    public void initialize() {
        if (mode == 0) {
            this.desiredheading = swerve.bestAngleToApproachNote();
            speedConstant = 3;
        } else if (mode == 1) {
            if (swerve.getHeading().getDegrees() < 0 && desiredheading.getDegrees() == 180) {
                desiredheading = Rotation2d.fromDegrees(-180);
            } else if (swerve.getHeading().getDegrees() > 0 && desiredheading.getDegrees() == -180) {
                desiredheading = Rotation2d.fromDegrees(180);
            }
        }
        
    
        //System.out.println("Desired location Initialize: " + desiredheading);

    }

    @Override
    public void execute() {
        double heading = swerve.getHeading().getDegrees();
        Rotation2d difference = desiredheading.minus(swerve.getHeading());
        // if (difference.getDegrees() > 180) {
        //     desiredheading = Rotation2d.fromDegrees(desiredheading.getDegrees() - 360);
        // } else if (difference.getDegrees() < -180) {
        //     desiredheading = Rotation2d.fromDegrees(desiredheading.getDegrees() + 360);
        // }
        if(desiredheading.getDegrees() > 180) {
            if (heading < desiredheading.getDegrees()-180) {
                desiredheading = new Rotation2d().plus(desiredheading);
            }
        } else if (desiredheading.getDegrees() < -180) {
            if (heading > desiredheading.getDegrees()+180) {
                desiredheading = new Rotation2d().plus(desiredheading);
            }
        }
        System.out.println("DesiredHeading: " + desiredheading.getDegrees());  
        difference = desiredheading.minus(swerve.getHeading());
        double turnSpeed = difference.getRadians()*speedConstant;
        //System.out.println("Desired location Execute: " + desiredheading);
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
        
        //System.out.println("Difference: " + Math.abs(desiredheading.getDegrees()-swerve.getHeading().getDegrees()));
        return Math.abs(desiredheading.getDegrees()-swerve.getHeading().getDegrees()) < 7;
        
    }


    @Override
    public void end(boolean interrupted) {
        
        swerve.drive(new Translation2d(0,0), 0, true);
        //System.out.println("End Direction Reached");
    }
}
