package frc.team7520.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team7520.robot.Constants;
import frc.team7520.robot.Constants.ShooterConstants.Position;
import frc.team7520.robot.subsystems.shooter.ShooterSubsystem;


public class AutoShootRest extends Command {
    private final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();

    public AutoShootRest() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.shooterSubsystem);
    }

    @Override
    public void initialize() {
        shooterSubsystem.setTurretPosition(Position.REST);
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

    }
}
