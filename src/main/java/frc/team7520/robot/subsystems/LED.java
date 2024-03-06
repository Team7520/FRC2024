package frc.team7520.robot.subsystems;


import java.lang.annotation.AnnotationTypeMismatchException;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix6.signals.Led2OnColorValue;
import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

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

    public final static CANdle candle = new CANdle(17);
    
    private final Animation idleAnimation = new RainbowAnimation(255, 0.75, 100);
    private final Animation intakingAnimation = new ColorFlowAnimation(255, 165, 0, 0, 1, 100, Direction.Forward);
    private final Animation noteIn = new StrobeAnimation(0, 255, 0, 0, 0.4, 100);


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
                candle.animate(idleAnimation);
            }
        );
    }

    public InstantCommand intaking() {
        return new InstantCommand(
            () -> {
                candle.animate(intakingAnimation);
            }
        );
    }

    public InstantCommand noteIn() {
        return new InstantCommand(
            () -> {
                candle.animate(noteIn);
            }
        );
    }
}

