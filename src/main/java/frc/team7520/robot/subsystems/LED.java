package frc.team7520.robot.subsystems;


import java.util.Optional;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team7520.robot.Constants.OperatorConstants;

public class LED extends SubsystemBase {

    // With eager singleton initialization, any static variables/fields used in the
    // constructor must appear before the "INSTANCE" variable so that they are initialized
    // before the constructor is called when the "INSTANCE" variable initializes.

    /**
     * The Singleton instance of this LED. Code should use
     * the {@link #getInstance()} method to get the single instance (rather
     * than trying to construct an instance of this class.)
     */
    private final static LED INSTANCE = new LED();

    private double m_Color = 0.0;
    private Spark m_ledController = new Spark(0);
    Optional<Alliance> alliance = DriverStation.getAlliance();
    private SensorSubsystem sensorSubsystem = SensorSubsystem.getInstance();

    private final XboxController operatorController =
            new XboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);

    /**
     * Returns the Singleton instance of this LED. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class. For example: {@code LED.getInstance();}
     */
    @SuppressWarnings("WeakerAccess")
    public static LED getInstance() {
        return INSTANCE;
    }

    /**
     * Creates a new instance of this LED. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    private LED() {
    }

    public void start() {
        m_Color = -0.99; // RAINBOW_RAINBOW_PALETTE
        m_ledController.set(m_Color);
    }

    private void black() {
        // alliance = DriverStation.getAlliance();

        // if (alliance.get() == Alliance.Red) {
        //     m_Color = -0.31; // LIGHT CHASE RED
        // }

        // else if (alliance.get() == Alliance.Blue) {
        //     m_Color = -0.29; // LIGHT CHASE BLUE
        // }
        m_Color = 0.99; // BLACK
    }

    private void green() {
        m_Color = 0.77; // GREEN
    }

    private void white() {
        m_Color = 0.93; // WHITE
    }

    private void red() {
        m_Color = 0.61; // RED
    }

    private void blue() {
        m_Color = 0.87; // BLUE
    }

    private void yellow() {
        m_Color = 0.69; // YELLOW
    }

    public Command idle() {
        return run(
            () -> {
                black();
            }
        );
    }

    public Command noteInIntake() {
        return run(
            () -> {
                green();
                new WaitCommand(0.25);
                black();
                new WaitCommand(0.25);
            }
        );
    }
    
    public Command noteInTurret() {
        return run(
            () -> {
                white();
                new WaitCommand(0.25);
                black();
                new WaitCommand(0.25);
            }
        );
    }

    public Command noTag() {
        return run(
            () -> {
                red();
                new WaitCommand(0.25);
                black();
                new WaitCommand(0.25);
            }
        );
    }

    public Command tagFound() {
        return run(
            () -> {
                blue();
                new WaitCommand(0.25);
                black();
                new WaitCommand(0.25);
            }
        );
    }

    public Command feeding() {
        return run(
            () -> {
                yellow();
                new WaitCommand(0.25);
                black();
                new WaitCommand(0.25);
            }
        );
    }

    @Override
    public void periodic() {
        int pov = operatorController.getPOV();

        if (sensorSubsystem.getBeamBreak()) {
            green();
        } else if (sensorSubsystem.getColorSensorProximity()) {
            white();
        } else if (pov == 90) {
            yellow();
        } else {
            black();
        }

        m_ledController.set(m_Color);
        // System.err.println(m_Color);
    }
}

