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
    private Spark m_ledController = new Spark(1);
    

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

    public void idle() {
        m_Color = -0.99; // RAINBOW_RAINBOW_PALETTE
        m_ledController.set(m_Color);
    }

    public void noteInIntake() {
        m_Color = 0.93; // WHITE
        m_ledController.set(m_Color);
    }

    public void noteInTurret() {
        m_Color = 0.77; // GREEN
        m_ledController.set(m_Color);
    }
}

