package frc.team7520.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team7520.robot.subsystems.LED;
import frc.team7520.robot.subsystems.SensorSubsystem;
import frc.team7520.robot.subsystems.shooter.ShooterSubsystem;

public class Sensor extends Command {
    private final SensorSubsystem sensorSubsystem;
    private final LED LED;

    public Sensor(SensorSubsystem sensorSubsystem, LED LED) {
        this.sensorSubsystem = sensorSubsystem;
        this.LED = LED;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(sensorSubsystem);
        addRequirements(LED);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        boolean noteInIntake = sensorSubsystem.getBeamBreak();
        int proximity = sensorSubsystem.getColorSensorProximity();
        if (proximity>150){
            LED.noteInTurret();
        } else if(noteInIntake){
            LED.noteInIntake();
        } else {
            LED.idle();
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
