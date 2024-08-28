package frc.team7520.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team7520.robot.Constants;
import frc.team7520.robot.subsystems.shooter.ShooterSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class AutoShoot extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final BooleanSupplier restSup;
    private final BooleanSupplier subwooferPosSup;
    private final BooleanSupplier wingLingPosSup;
    private final BooleanSupplier podiumPosSup;
    private final DoubleSupplier throttleSup;

    public AutoShoot(ShooterSubsystem shooterSubsystem, BooleanSupplier restSup, BooleanSupplier subwooferPosSup, BooleanSupplier wingLingPosSup, BooleanSupplier podiumPosSup, DoubleSupplier throttleSup) {
        this.shooterSubsystem = shooterSubsystem;
        this.restSup = restSup;
        this.subwooferPosSup = subwooferPosSup;
        this.wingLingPosSup = wingLingPosSup;
        this.podiumPosSup = podiumPosSup;
        this.throttleSup = throttleSup;
        
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(shooterSubsystem);
    }

    // public void shootSpeed() {
    //     if (restSup.getAsBoolean()) {
    //         shooterSubsystem.setSpeed(Constants.ShooterConstants.Position.REST.getSpeed(), false);
    //         return;
    //     }
    //     if (subwooferPosSup.getAsBoolean()) {
    //         shooterSubsystem.setSpeed(Constants.ShooterConstants.Position.SUBWOOFER.getSpeed(), false);
    //         return;
    //     }
    //     if (wingLingPosSup.getAsBoolean()) {
    //         shooterSubsystem.setSpeed(Constants.ShooterConstants.Position.WINGLINE.getSpeed(), false);
    //         return;
    //     }
    //     if (podiumPosSup.getAsBoolean()) {
    //         shooterSubsystem.setSpeed(Constants.ShooterConstants.Position.PODIUM.getSpeed(), false);
    //         return;
    //     }
    //     shooterSubsystem.stopShooting();
    // }

    public void pivotPosition() {
        if (restSup.getAsBoolean()) {
            shooterSubsystem.setPivotPosition(Constants.ShooterConstants.Position.REST);
            shooterSubsystem.setTraversePosition(Constants.ShooterConstants.Position.REST);
            return;
        }
        if (subwooferPosSup.getAsBoolean()) {
            shooterSubsystem.setPivotPosition(Constants.ShooterConstants.Position.SUBWOOFER);
            shooterSubsystem.setTraversePosition(Constants.ShooterConstants.Position.SUBWOOFER);
            return;
        }
        if (wingLingPosSup.getAsBoolean()) {
            shooterSubsystem.setPivotPosition(Constants.ShooterConstants.Position.WINGLINE);
            shooterSubsystem.setTraversePosition(Constants.ShooterConstants.Position.WINGLINE);
            return;
        }
        if (podiumPosSup.getAsBoolean()) {
            shooterSubsystem.setPivotPosition(Constants.ShooterConstants.Position.PODIUM);
            shooterSubsystem.setTraversePosition(Constants.ShooterConstants.Position.PODIUM);
            return;
        }

    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        pivotPosition();
        double throttle = throttleSup.getAsDouble()*0.9;
        shooterSubsystem.setSpeed(throttle, false);  

        SmartDashboard.putBoolean("RestSup", restSup.getAsBoolean());
        SmartDashboard.putBoolean("SubwooferSup", subwooferPosSup.getAsBoolean());
        SmartDashboard.putBoolean("WingSup", wingLingPosSup.getAsBoolean());
        SmartDashboard.putBoolean("PodSup", podiumPosSup.getAsBoolean());
        // SmartDashboard.putBoolean("test", true);
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
