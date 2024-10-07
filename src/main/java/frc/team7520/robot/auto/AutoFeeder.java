package frc.team7520.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team7520.robot.subsystems.intake.IntakeSubsystem;
import frc.team7520.robot.subsystems.shooter.ShooterSubsystem;

public class AutoFeeder extends Command {
    private final IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();
    private final double ringSpdSup;
    private final double feederSpdSup;

    public AutoFeeder(double ringSpdSup, double feederSpdSup) {
        this.ringSpdSup = ringSpdSup;
        this.feederSpdSup = feederSpdSup;
        
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.setRingSpeed(ringSpdSup);
        intakeSubsystem.setFeederSpeed(feederSpdSup);
    }

    @Override
    public void execute() {
        
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setRingSpeed(0);
        intakeSubsystem.setFeederSpeed(0);
    }
}
