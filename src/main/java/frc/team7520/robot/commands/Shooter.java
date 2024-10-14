package frc.team7520.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team7520.robot.Constants;
import frc.team7520.robot.subsystems.shooter.ShooterSubsystem;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Shooter extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final DoubleSupplier throttleSup;
    private final BooleanSupplier pivotDown;
    private final BooleanSupplier pivotUp;

    public Shooter(ShooterSubsystem shooterSubsystem, DoubleSupplier throttleSup, BooleanSupplier pivotDown, BooleanSupplier pivotUp) {
        this.shooterSubsystem = shooterSubsystem;
        this.throttleSup = throttleSup;
        this.pivotDown = pivotDown;
        this.pivotUp = pivotUp;


        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(shooterSubsystem);
    }


    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
//        turretPosition();
        double throttle = throttleSup.getAsDouble();

        shooterSubsystem.aimAtTarget(shooterSubsystem.getSpeakerTranslation());
        shooterSubsystem.setSpeed(throttle, false);
        if (pivotUp.getAsBoolean()){
            shooterSubsystem.setPivotPosition(shooterSubsystem.getPivotEncoder().plus(Rotation2d.fromDegrees(2)));
        }

        if (pivotDown.getAsBoolean()){
            shooterSubsystem.setPivotPosition(shooterSubsystem.getPivotEncoder().minus(Rotation2d.fromDegrees(1)));
        }
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
