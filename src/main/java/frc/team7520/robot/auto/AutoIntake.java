package frc.team7520.robot.auto;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team7520.robot.Constants;
import frc.team7520.robot.subsystems.SensorSubsystem;
import frc.team7520.robot.subsystems.intake.IntakeSubsystem;

public class AutoIntake extends Command {
    private final IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();
    private final SensorSubsystem sensorSubsystem = SensorSubsystem.getInstance();
    private final double groundSpdSup;
    private final double ringSpdSup;
    private final double feederSpdSup;

    public AutoIntake(double groundSpdSup, double ringSpdSup, double feederSpdSup) {
        this.groundSpdSup = groundSpdSup;
        this.ringSpdSup = ringSpdSup;
        this.feederSpdSup = feederSpdSup;
        
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.setSpeed(groundSpdSup);
        intakeSubsystem.setRingSpeed(ringSpdSup);
        intakeSubsystem.setFeederSpeed(feederSpdSup);
        SmartDashboard.putBoolean("AutoIntakeCmd", true);
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
        intakeSubsystem.setSpeed(0);
        intakeSubsystem.setRingSpeed(0);
        intakeSubsystem.setFeederSpeed(0);
        SmartDashboard.putBoolean("AutoIntakeCmd", false);
    }
}
