package frc.team7520.robot.subsystems;


import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

    public Command idle() {
        return run(
            () -> {
                m_Color = 0.61; // RED
                m_ledController.set(m_Color);
            }
        );
    }

    public InstantCommand noteInIntake() {
        return new InstantCommand(
            () -> {
                m_Color = 0.87; // BLUE
                m_ledController.set(m_Color);
            }
        );
    }

    public InstantCommand noteInTurret() {
        return new InstantCommand(
            () -> {
                m_Color = 0.77; // GREEN
                m_ledController.set(m_Color);
            }
        );
    }
}

