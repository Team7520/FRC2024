package frc.team7520.robot.subsystems.shooter;


import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team7520.robot.Constants;
import frc.team7520.robot.Constants.ShooterConstants;
import frc.team7520.robot.Constants.ShooterConstants.PivotConstants;
import frc.team7520.robot.Constants.ShooterConstants.TraverseConstants;
import frc.team7520.robot.subsystems.swerve.SwerveSubsystem;
import lombok.Getter;
import lombok.Setter;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import swervelib.SwerveController;

import static frc.team7520.robot.Constants.aprilTagFieldLayout;

@Getter
@Setter
public class ShooterSubsystem extends SubsystemBase {

    // With eager singleton initialization, any static variables/fields used in the
    // constructor must appear before the "INSTANCE" variable so that they are initialized
    // before the constructor is called when the "INSTANCE" variable initializes.

    private final TalonFX shooterMotorBot;
    private final TalonFX shooterMotorTop;
    private final TalonFX pivotMotor;

    private final TalonFX traverseMotor;

    public Rotation2d desiredPivotPos = new Rotation2d(0);
    public Rotation2d desiredTraversePos = new Rotation2d(0);

    private final SlewRateLimiter speedLimiter = new SlewRateLimiter(100);

    private final SwerveSubsystem swerveDrive = SwerveSubsystem.getInstance();

    /**
     * The Singleton instance of this shooterSubsystem. Code should use
     * the {@link #getInstance()} method to get the single instance (rather
     * than trying to construct an instance of this class.)
     */
    private final static ShooterSubsystem INSTANCE = new ShooterSubsystem();

    /**
     * Returns the Singleton instance of this shooterSubsystem. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class. For example: {@code shooterSubsystem.getInstance();}
     */
    @SuppressWarnings("WeakerAccess")
    public static ShooterSubsystem getInstance() {
        return INSTANCE;
    }

    private void configPivot(TalonFX pivotMotor) {
        var tlnfxConfigs = new TalonFXConfiguration();
        var motorConfigs = new MotorOutputConfigs();
        var slot0Configs = new Slot0Configs();
        var motionMagicConfigs = tlnfxConfigs.MotionMagic;

        pivotMotor.getConfigurator().apply(new TalonFXConfiguration());
        motorConfigs.NeutralMode = PivotConstants.neutralMode;
        tlnfxConfigs.Feedback.SensorToMechanismRatio = PivotConstants.degreeConversionFactor;

        slot0Configs.kP = PivotConstants.kP;
        slot0Configs.kI = PivotConstants.kI;
        slot0Configs.kD = PivotConstants.kD;
        slot0Configs.kG = PivotConstants.kG;
        slot0Configs.kS = PivotConstants.kS;
        slot0Configs.kV = PivotConstants.kV;
        slot0Configs.kA = PivotConstants.kA;

        motionMagicConfigs.MotionMagicCruiseVelocity = PivotConstants.motionMagicVelocity;
        motionMagicConfigs.MotionMagicAcceleration = PivotConstants.motionMagicAccel;
        motionMagicConfigs.MotionMagicJerk = PivotConstants.motionMagicJerk;

        pivotMotor.getConfigurator().apply(motorConfigs);
        pivotMotor.getConfigurator().apply(tlnfxConfigs);
        pivotMotor.getConfigurator().apply(slot0Configs);
        pivotMotor.getConfigurator().apply(motionMagicConfigs);

        pivotMotor.setInverted(true);

        pivotMotor.setPosition(0);
    }

    private void configTraverse(TalonFX traverseMotor) {
        var tlnfxConfigs = new TalonFXConfiguration();
        var motorConfigs = new MotorOutputConfigs();
        var slot0Configs = new Slot0Configs();
        var motionMagicConfigs = tlnfxConfigs.MotionMagic;

        traverseMotor.getConfigurator().apply(new TalonFXConfiguration());
        motorConfigs.NeutralMode = TraverseConstants.neutralMode;
        tlnfxConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        tlnfxConfigs.Feedback.SensorToMechanismRatio = TraverseConstants.degreeConversionFactor;

        slot0Configs.kP = TraverseConstants.kP;
        slot0Configs.kI = TraverseConstants.kI;
        slot0Configs.kD = TraverseConstants.kD;
        slot0Configs.kG = TraverseConstants.kG;
        slot0Configs.kS = TraverseConstants.kS;
        slot0Configs.kV = TraverseConstants.kV;
        slot0Configs.kA = TraverseConstants.kA;

        motionMagicConfigs.MotionMagicCruiseVelocity = TraverseConstants.motionMagicVelocity;
        motionMagicConfigs.MotionMagicAcceleration = TraverseConstants.motionMagicAccel;
        motionMagicConfigs.MotionMagicJerk = TraverseConstants.motionMagicJerk;

        traverseMotor.getConfigurator().apply(motorConfigs);
        traverseMotor.getConfigurator().apply(tlnfxConfigs);
        traverseMotor.getConfigurator().apply(slot0Configs);
        traverseMotor.getConfigurator().apply(motionMagicConfigs);

        traverseMotor.setInverted(true);

        traverseMotor.setPosition(0);
    }

    /**
     * Creates a new instance of this shooterSubsystem. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    private ShooterSubsystem() {
        pivotMotor = new TalonFX(PivotConstants.CAN_ID);
        traverseMotor = new TalonFX(TraverseConstants.CAN_ID);

        configPivot(pivotMotor);
        configTraverse(traverseMotor);

        shooterMotorBot = new TalonFX(ShooterConstants.shooterBotID); // left = bot
        shooterMotorTop = new TalonFX(ShooterConstants.shooterTopID);

        shooterMotorBot.setInverted(true);
        shooterMotorTop.setInverted(true);
    }


    public Rotation2d getPivotEncoder() {
        return Rotation2d.fromRotations(pivotMotor.getPosition().getValueAsDouble());
    }

    public Rotation2d getTraverseEncoder() {
        return Rotation2d.fromRotations(traverseMotor.getPosition().getValueAsDouble());
    }

    public Rotation3d getTurretPosition() {
        return new Rotation3d(0, getPivotEncoder().getRadians(), getTraverseEncoder().getRadians());
    }

    public void setSpeed(double speed, boolean closedLoop) {
        speed = speedLimiter.calculate(speed);

        if (closedLoop) {
            speed *= Constants.ShooterConstants.MAX_RPM;

            // TODO: Implement closed loop control
        } else {
            shooterMotorBot.set(speed);
            shooterMotorTop.set(speed);
        }
    }


    public void setPivotPosition(Rotation2d desiredPivotPos) {
        // final PositionVoltage moveRequest = new PositionVoltage(0).withSlot(0);
        final MotionMagicVoltage moveRequest = new MotionMagicVoltage(0).withSlot(0);
        pivotMotor.setControl(moveRequest.withPosition(desiredPivotPos.getDegrees()));
        SmartDashboard.putNumber("desiredPivotPos", desiredPivotPos.getDegrees());
    }

    public void setTraversePosition(Rotation2d desiredTraversePos) {
        // final PositionVoltage moveRequest = new PositionVoltage(0).withSlot(0);
        final MotionMagicVoltage moveRequest = new MotionMagicVoltage(0).withSlot(0);
        traverseMotor.setControl(moveRequest.withPosition(desiredTraversePos.getDegrees()));
        SmartDashboard.putNumber("desiredTraversePos", desiredTraversePos.getDegrees());
    }

    public void setTurretPosition(ShooterConstants.Position desiredTurretPos) {
        setPivotPosition(desiredTurretPos.getPivot());
        setTraversePosition(desiredTurretPos.getTraverse());
    }

    public InstantCommand stopShooting() {
        return new InstantCommand(() -> {
            shooterMotorBot.stopMotor();
            shooterMotorTop.stopMotor();
        });
    }


    /**
     * Get the distance to the speaker.
     *
     * @return Distance to speaker in meters.
     */
    public double getDistanceToSpeaker()
    {
        int allianceAprilTag = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? 7 : 4;
        // Taken from PhotonUtils.getDistanceToPose
        Pose3d speakerAprilTagPose = aprilTagFieldLayout.getTagPose(allianceAprilTag).get();
        return swerveDrive.getPose().getTranslation().getDistance(speakerAprilTagPose.toPose2d().getTranslation());
    }

    /**
     * Get the yaw to aim at the speaker.
     *
     * @return {@link Rotation2d} of which you need to achieve.
     */
    public Rotation2d getSpeakerYaw()
    {
        int allianceAprilTag = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? 7 : 4;
        // Taken from PhotonUtils.getYawToPose()
        Pose3d        speakerAprilTagPose = aprilTagFieldLayout.getTagPose(allianceAprilTag).get();
        Translation2d relativeTrl         = speakerAprilTagPose.toPose2d().relativeTo(swerveDrive.getPose()).getTranslation();
        return new Rotation2d(relativeTrl.getX(), relativeTrl.getY()).plus(swerveDrive.getHeading());
    }

    public Translation3d getSpeakerTranslation()
    {
        int allianceAprilTag = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? 7 : 4;
        // Taken from PhotonUtils.getYawToPose()
        Pose3d        speakerAprilTagPose = aprilTagFieldLayout.getTagPose(allianceAprilTag).get();
        return speakerAprilTagPose.relativeTo(new Pose3d(swerveDrive.getPose())).getTranslation();
    }

    /**
     * Aim the robot at the speaker.
     *
     * @param tolerance Tolerance in degrees.
     * @return Command to turn the robot to the speaker.
     */
    public Command aimAtSpeaker(double tolerance)
    {
        return run(
                () -> {
                    aimAtTarget(getSpeakerTranslation());
                });
    }

    /**
     * Aim the robot at a target in the form of a {@link edu.wpi.first.math.geometry.Translation3d}.
     * Uses Quadratic Regression to aim at the target.
     *
     * @param target {@link edu.wpi.first.math.geometry.Transform3d} to communicate with.
     */
    public void aimAtTarget(Translation3d target){
        // TODO: Read datapoints and calculate the quadratic regression.
        // For now we will just aim at the target.
//        Rotation2d targetYaw = Rotation2d.fromRadians(target.getZ());
//        Rotation2d targetPitch = Rotation2d.fromRadians(target.getY());

        Rotation2d targetYaw = new Rotation2d(target.getX(), target.getY());
        Rotation2d targetPitch = new Rotation2d(target.getX(), target.getZ());
        // Aim at the target.
        setTraversePosition(targetYaw);
        setPivotPosition(targetPitch);

    }



    @Override
    public void periodic() {


        SmartDashboard.putNumber("pivotEncoder", getPivotEncoder().getDegrees());
        SmartDashboard.putNumber("pivotVel", pivotMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("pivotVoltage", pivotMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("pivotCurr", pivotMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("pivotClosedLoopError", pivotMotor.getClosedLoopError().getValueAsDouble());
        SmartDashboard.putNumber("traverseEncoder", getTraverseEncoder().getDegrees());
        SmartDashboard.putNumber("traverseVel", traverseMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("traverseVoltage", traverseMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("traverseCurr", traverseMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("traverseClosedLoopError", traverseMotor.getClosedLoopError().getValueAsDouble());
    }
}

