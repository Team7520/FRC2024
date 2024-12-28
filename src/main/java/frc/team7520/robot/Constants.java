// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team7520.robot;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import lombok.Getter;
import swervelib.math.Matter;
import swervelib.parser.PIDFConfig;
import swervelib.telemetry.SwerveDriveTelemetry;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final double ROBOT_MASS = 49.8952; // Mass in kilos
    public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    public static final double LOOP_TIME = 0.13; //s, 20ms + 110ms sprk max velocity lag
    public static final double MAX_SPEED  = Units.feetToMeters(16);

    public static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    public static final class LimeLightConstants {
        public static final String name = "limelight";
    };

    public static final class Auton {

        public static final PIDFConfig TranslationPID = new PIDFConfig(0.7, 0, 0);
        public static final PIDFConfig angleAutoPID = new PIDFConfig(0.4, 0, 0.01);

        public static final double MAX_ACCELERATION = 2;
    }

    public static final class Drivebase {

        // Hold time on motor brakes when disabled
        public static final double WHEEL_LOCK_TIME = 10; // seconds
    }

    public static class OperatorConstants {
        // Joystick Ports
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;

        // Joystick Deadband
        public static final double LEFT_X_DEADBAND = 0.01;
        public static final double LEFT_Y_DEADBAND = 0.01;
        public static final double RIGHT_X_DEADBAND = 0.01;
        public static final double TURN_CONSTANT = 0.75;
    }

    public static class Swerve {
        public static final double DRIVE_GEAR_RATIO = 6.12;
        public static final double ANGLE_GEAR_RATIO = 150/7d;
        /*
         * Previous swervebase IDs and offsets
         * Intake was previously back
         */
        public class oldSwerveBL {
            double drv = 8;
            double ang = 9;
            double enc = 13;
            double encOff = 34.45308;
        }
        public class oldSwerveBR {
            double drv = 2;
            double ang = 3;
            double enc = 10;
            double encOff = 341.5428;
        }
        public class oldSwerveFL {
            double drv = 6;
            double ang = 7;
            double enc = 12;
            double encOff = 184.48236;
        }
        public class oldSwerveFR {
            double drv = 4;
            double ang = 5;
            double enc = 11;
            double encOff = 45.9666;
        }
    }

    public static class Telemetry {

        // change at comp to low
        public static final SwerveDriveTelemetry.TelemetryVerbosity SWERVE_VERBOSITY = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;
    }
    
    public static final class Vision{

        public static final Vector<N3> stdDevs = VecBuilder.fill(0.2, 0.2, 0.2);

    }
}
