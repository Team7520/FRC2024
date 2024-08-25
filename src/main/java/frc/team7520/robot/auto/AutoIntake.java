/*package frc.team7520.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team7520.robot.Constants;
import frc.team7520.robot.subsystems.intake.IntakeSubsystem;


public class AutoIntake extends Command {
//    private final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
    private final IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();

    private final Constants.IntakeConstants.Position desiredPos;

    public AutoIntake(Constants.IntakeConstants.Position desiredPos) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(
//                this.shooterSubsystem,
                this.intakeSubsystem
        );

        this.desiredPos = desiredPos;
    }

    @Override
    public void initialize() {
        intakeSubsystem.setPosition(desiredPos);
//        shooterSubsystem.setSpeed(1, false);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return intakeSubsystem.atPosition();
    }

    @Override
    public void end(boolean interrupted) {

    }
}
*/