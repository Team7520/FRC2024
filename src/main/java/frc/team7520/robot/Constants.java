// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team7520.robot;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
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

    public static class IntakeConstants {
        public enum Position {
            SHOOT(new Rotation2d(0), 1),
            INTAKE(new Rotation2d(Units.degreesToRadians(212.152317734808d)), -0.3),
            AMP(Rotation2d.fromDegrees(83.5138969421), 0.495);

            private final Rotation2d position;
            private final double speed;

            Position(Rotation2d position, double speed) {
                this.position = position;
                this.speed = speed;
            }

            public Rotation2d getPosition() {
                return position;
            }

            public double getSpeed() {
                return speed;
            }
        }

        public static class WheelConstants {
            public static final int IntakeID = 40;
            public static final int RingOneID = 14;
            public static final int RingTwoID = 15;
            public static final int FeederID = 42;
            public static final double kP = 0.0020645;
            public static final double kI = 0;
            public static final double kD = 0;
            public static final double kFF = 0;

            public static final double MAX_RPM = 5676;
        }
    }

    public static class ShooterConstants {
        public static final int shooterBotID = 20;
        public static final int shooterTopID = 19;
        public static final int ampMechID = 25;
        public static final double kP = 0.002;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kFF = 0.000156;

        public static final double MAX_RPM = 5676;

        public static final int colourSensorSensedProximity = 145;


        @Getter
        public enum Position {
            // Use Rotation2d.fromDegrees for more intuitive degree handling
            REST(0, 0), // DO NOT CHANGE
            DRIVE(0, -95),
            SUBWOOFERCENTER(77.6, 0),
            SUBWOOFERLEFT(77.6, 95),
            SUBWOOFERRIGHT(77.6, -95),
            WINGLINERED(35, 42.3),
            WINGLINEBLUE(35, -42.3),
            PODIUMRED(44.9316, 25),
            PODIUMBLUE(44.9316, -25),
            NOTECENTER(51.9326, 0),
            AMP(72.4453, 0);

            private final Rotation2d pivot;
            private final Rotation2d traverse;

            // Overloaded constructor to accept doubles in degrees
            Position(double pivotDegrees, double traverseDegrees) {
                this.pivot = Rotation2d.fromDegrees(pivotDegrees);
                this.traverse = Rotation2d.fromDegrees(traverseDegrees);
            }
        }


        public static class PivotConstants {
            public static final int CAN_ID = 21;

            public static final double gearRatio = 80.0/3;
            public static final double degreeConversionFactor = 1/gearRatio;
            public static final NeutralModeValue neutralMode = NeutralModeValue.Coast;
            public static final double kP = 0.65;
            public static final double kI = 0;
            public static final double kD = 0;
            public static final double kG = 0;
            public static final double kS = 0;
            public static final double kV = 0;
            public static final double kA = 0;

            public static final double motionMagicVelocity = 450;
            public static final double motionMagicAccel = 800;
            public static final double motionMagicJerk = 2000;
        }

        public static class TraverseConstants {
            public static final int CAN_ID = 16;

            public static final double gearRatio = 14/160.0;
            public static final double degreeConversionFactor = 1/(gearRatio);
            public static final NeutralModeValue neutralMode = NeutralModeValue.Coast;
            public static final double kP = 80;
            public static final double kI = 0;
            public static final double kD = 0;
            public static final double kG = 0;
            public static final double kS = 0;
            public static final double kV = 0;
            public static final double kA = 0;

            public static final double motionMagicVelocity = 10;
            public static final double motionMagicAccel = 200/50;
            public static final double motionMagicJerk = 2000/3;
        }
    }

    public static class ClimberConstants {
        public static final int climberLeftID = 30;
        public static final int climberRightID = 31;
        public static final int maxPosition = 520;

        public static final double kP = 0.004;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kFF = 0.0006;

    }
    public static class AmpConstants {
        public enum Position {
            REST(new Rotation2d(0), 1),
            AMP(new Rotation2d(Units.degreesToRadians(83.5138969421)), 0.495);

            private final Rotation2d position;
            private final double speed;

            Position(Rotation2d position, double speed) {
                this.position = position;
                this.speed = speed;
            }

            public Rotation2d getPosition() {
                return position;
            }

            public double getSpeed() {
                return speed;
            }
        }
        public static final int CAN_ID = 62;
        public static final double GearRatio = 100;
        public static final double degreeConversionFactor = 360/GearRatio;
        public static final double rotationConversionFactor = 1/GearRatio;

        public static final double Rest = 0;
        public static final double Amp = 135;

        public static final double kP = 0.00022;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kFF = 0.000156;

        public static final double OUTPUT_MAX = 1;
        public static final double OUTPUT_MIN = -1;

        public static final double SmartMaxVel = 6000; //600000
        public static final double SmartMinVel = 0;
        public static final double SmartAccel = 100; //10000
        public static final double SmartErr = 2;
        public static final int SlotID = 0;

    }
}
