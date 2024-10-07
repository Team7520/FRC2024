package frc.team7520.robot.commands;

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
    private final BooleanSupplier feederSup;
    private final BooleanSupplier podiumPosSup;
    private final DoubleSupplier throttleSup;
    private final DoubleSupplier povSup;
    private final BooleanSupplier noteSup;
    private final BooleanSupplier drivePosSup;

    public Shooter(ShooterSubsystem shooterSubsystem, DoubleSupplier povSup, BooleanSupplier feederSup, BooleanSupplier podiumPosSup, DoubleSupplier throttleSup, BooleanSupplier noteSup, BooleanSupplier drivePosSup) {
        this.shooterSubsystem = shooterSubsystem;
        this.feederSup = feederSup;
        this.podiumPosSup = podiumPosSup;
        this.throttleSup = throttleSup;
        this.povSup = povSup;
        this.noteSup = noteSup;
        this.drivePosSup = drivePosSup;
        
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(shooterSubsystem);
    }

    public void turretPosition() {
        Optional<Alliance> alliance = DriverStation.getAlliance();

        if (povSup.getAsDouble() == 180) {
            shooterSubsystem.setPivotPosition(Constants.ShooterConstants.Position.REST);
            shooterSubsystem.setTraversePosition(Constants.ShooterConstants.Position.REST);
            return;
        }
        if (povSup.getAsDouble() == 0) {
            shooterSubsystem.setPivotPosition(Constants.ShooterConstants.Position.SUBWOOFERCENTER);
            shooterSubsystem.setTraversePosition(Constants.ShooterConstants.Position.SUBWOOFERCENTER);
            return;
        }
        if (povSup.getAsDouble() == 270) {
            shooterSubsystem.setPivotPosition(Constants.ShooterConstants.Position.SUBWOOFERLEFT);
            shooterSubsystem.setTraversePosition(Constants.ShooterConstants.Position.SUBWOOFERLEFT);
            return;
        }
        if (povSup.getAsDouble() == 90) {
            shooterSubsystem.setPivotPosition(Constants.ShooterConstants.Position.SUBWOOFERRIGHT);
            shooterSubsystem.setTraversePosition(Constants.ShooterConstants.Position.SUBWOOFERRIGHT);
            return;
        }
        // if (wingLingPosSup.getAsBoolean()) {
        //     if (alliance.get() == Alliance.Red) {
        //         shooterSubsystem.setPivotPosition(Constants.ShooterConstants.Position.WINGLINERED);
        //         shooterSubsystem.setTraversePosition(Constants.ShooterConstants.Position.WINGLINERED);
        //     }

        //     else if (alliance.get() == Alliance.Blue) {
        //         shooterSubsystem.setPivotPosition(Constants.ShooterConstants.Position.WINGLINEBLUE);
        //         shooterSubsystem.setTraversePosition(Constants.ShooterConstants.Position.WINGLINEBLUE);
        //     }

        //     return;
        // }
        if (podiumPosSup.getAsBoolean()) {
            if (alliance.get() == Alliance.Red){
                shooterSubsystem.setPivotPosition(Constants.ShooterConstants.Position.PODIUMRED);
                shooterSubsystem.setTraversePosition(Constants.ShooterConstants.Position.PODIUMRED);
            }

            else if (alliance.get() == Alliance.Blue) {
                shooterSubsystem.setPivotPosition(Constants.ShooterConstants.Position.PODIUMBLUE);
                shooterSubsystem.setTraversePosition(Constants.ShooterConstants.Position.PODIUMBLUE);
            }
            return;
        }

        if (noteSup.getAsBoolean()) {
            shooterSubsystem.setPivotPosition(Constants.ShooterConstants.Position.NOTECENTER);
            shooterSubsystem.setTraversePosition(Constants.ShooterConstants.Position.NOTECENTER);
            return;
        }

        if (drivePosSup.getAsBoolean()) {
            shooterSubsystem.setPivotPosition(Constants.ShooterConstants.Position.DRIVE);
            shooterSubsystem.setTraversePosition(Constants.ShooterConstants.Position.DRIVE);
            return;
        }

    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        turretPosition();
        double throttle = throttleSup.getAsDouble();
        if (feederSup.getAsBoolean()) {
            throttle = 0.65;
        }
        shooterSubsystem.setSpeed(throttle, false);  

        // SmartDashboard.putBoolean("WingSup", wingLingPosSup.getAsBoolean());
        SmartDashboard.putBoolean("PodSup", podiumPosSup.getAsBoolean());
        SmartDashboard.putNumber("POVNum", povSup.getAsDouble());
        SmartDashboard.putBoolean("noteSup", noteSup.getAsBoolean());
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
