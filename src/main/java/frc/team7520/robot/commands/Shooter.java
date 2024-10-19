package frc.team7520.robot.commands;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team7520.robot.Constants;
import frc.team7520.robot.subsystems.LED;
import frc.team7520.robot.subsystems.shooter.ShooterSubsystem;
import frc.team7520.robot.subsystems.swerve.Vision;

import java.util.Vector;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Shooter extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final DoubleSupplier throttleSup;
    private final BooleanSupplier turretRest;
    private final BooleanSupplier turretTrack;
    private final DoubleSupplier POVSup;

    private boolean upLock = false;
    private boolean downLock = false;

    public Shooter(ShooterSubsystem shooterSubsystem, DoubleSupplier throttleSup, BooleanSupplier turretRest, BooleanSupplier turretTrack, DoubleSupplier POVSup) {
        this.shooterSubsystem = shooterSubsystem;
        this.throttleSup = throttleSup;
        this.turretRest = turretRest;
        this.turretTrack = turretTrack;
        this.POVSup = POVSup;


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

//        shooterSubsystem.aimAtTarget(shooterSubsystem.getSpeakerTranslation());
        shooterSubsystem.setSpeed(throttle, false);
        if (turretTrack.getAsBoolean()){
            Vision.Cameras.SHOOTER_CAMERA.singleTagStdDevs.set(2, 0, Double.MAX_VALUE);
            Vision.Cameras.SHOOTER_CAMERA.multiTagStdDevs.set(2, 0, Double.MAX_VALUE);
//            shooterSubsystem.setPivotPosition(shooterSubsystem.getPivotEncoder().plus(Rotation2d.fromDegrees(2)));
            shooterSubsystem.aimAtTarget(shooterSubsystem.getSpeakerTranslation());
        }

        if (turretRest.getAsBoolean()){
            Vision.Cameras.SHOOTER_CAMERA.singleTagStdDevs.set(2, 0, 0.2);
            Vision.Cameras.SHOOTER_CAMERA.multiTagStdDevs.set(2, 0, 0.2);
//            shooterSubsystem.setPivotPosition(shooterSubsystem.getPivotEncoder().minus(Rotation2d.fromDegrees(1)));
            shooterSubsystem.setTurretPosition(Constants.ShooterConstants.Position.REST);
        }

        if(POVSup.getAsDouble() == 0){
            if(!upLock){
                shooterSubsystem.setPivotPosition(shooterSubsystem.getPivotEncoder().plus(Rotation2d.fromDegrees(2)));
                upLock = true;
            }
        }else {
            upLock = false;
        }

        if(POVSup.getAsDouble() == 180){
            if(!downLock){
                shooterSubsystem.setPivotPosition(shooterSubsystem.getPivotEncoder().minus(Rotation2d.fromDegrees(1)));
                downLock = true;
            }
        }else {
            downLock = false;
        }

        if(POVSup.getAsDouble() == 90){
            Vision.Cameras.SHOOTER_CAMERA.singleTagStdDevs.set(2, 0, Double.MAX_VALUE);
            Vision.Cameras.SHOOTER_CAMERA.multiTagStdDevs.set(2, 0, Double.MAX_VALUE);
            var target = shooterSubsystem.getFeedTranslation();
            shooterSubsystem.setTraversePosition(new Rotation2d(target.getX(), target.getY()));
            shooterSubsystem.setPivotPosition(Rotation2d.fromDegrees(40));
            LED.getInstance().feeding();
        }

        if (POVSup.getAsDouble() == 270){

            Vision.Cameras.SHOOTER_CAMERA.singleTagStdDevs.set(2, 0, 0.2);
            Vision.Cameras.SHOOTER_CAMERA.multiTagStdDevs.set(2, 0, 0.2);
//            shooterSubsystem.setPivotPosition(shooterSubsystem.getPivotEncoder().minus(Rotation2d.fromDegrees(1)));
            shooterSubsystem.setTurretPosition(Constants.ShooterConstants.Position.REST180);
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
