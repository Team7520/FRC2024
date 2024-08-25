package frc.team7520.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team7520.robot.subsystems.LED;
import frc.team7520.robot.subsystems.SensorSubsystem;
import frc.team7520.robot.subsystems.shooter.ShooterSubsystem;

public class Sensor extends Command {
    private final SensorSubsystem sensorSubsystem;
    private final LED LED;
    private final BooleanSupplier isInIntake;
    private final IntSupplier proximity;

    public Sensor(SensorSubsystem sensorSubsystem, LED LED, BooleanSupplier isInIntake, IntSupplier proximity) {
        this.sensorSubsystem = sensorSubsystem;
        this.LED = LED;
        this.isInIntake = isInIntake;
        this.proximity = proximity;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(sensorSubsystem);
        addRequirements(LED);
    }

    @Override
    public void initialize() {
        LED.idle();
    }

    @Override
    public void execute() {
        if (proximity.getAsInt()>150){
            LED.noteInTurret();
        } else if(isInIntake.getAsBoolean()){
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
