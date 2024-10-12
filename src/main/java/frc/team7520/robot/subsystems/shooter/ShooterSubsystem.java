package frc.team7520.robot.subsystems.shooter;


import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team7520.robot.Constants;
import frc.team7520.robot.Constants.ShooterConstants;
import frc.team7520.robot.Constants.ShooterConstants.PivotConstants;
import frc.team7520.robot.Constants.ShooterConstants.TraverseConstants;
import lombok.Getter;
import lombok.Setter;

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

    public Rotation2d getDesiredPivotPosition() {
        return desiredPivotPos;
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

    public void setPivotPosition(ShooterConstants.Position position) {
        desiredPivotPos = position.getPivot();
        // final PositionVoltage moveRequest = new PositionVoltage(0).withSlot(0);
        final MotionMagicVoltage moveRequest = new MotionMagicVoltage(0).withSlot(0);
        pivotMotor.setControl(moveRequest.withPosition(desiredPivotPos.getDegrees()));
    }

    public void setTraversePosition(ShooterConstants.Position position) {
        desiredTraversePos = position.getTraverse();
        // final PositionVoltage moveRequest = new PositionVoltage(0).withSlot(0);
        final MotionMagicVoltage moveRequest = new MotionMagicVoltage(0).withSlot(0);
        traverseMotor.setControl(moveRequest.withPosition(desiredTraversePos.getDegrees()));
    }

    public InstantCommand stopShooting() {
        return new InstantCommand(() -> {
            shooterMotorBot.stopMotor();
            shooterMotorTop.stopMotor();
        });
    }


    @Override
    public void periodic() {


        SmartDashboard.putNumber("pivotEncoder", getPivotEncoder());
        SmartDashboard.putNumber("pivotVel", pivotMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("pivotVoltage", pivotMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("pivotCurr", pivotMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("desiredPivotPos", desiredPivotPos.getDegrees());
        SmartDashboard.putNumber("pivotClosedLoopError", pivotMotor.getClosedLoopError().getValueAsDouble());
        SmartDashboard.putNumber("traverseEncoder", getTraverseEncoder());
        SmartDashboard.putNumber("traverseVel", traverseMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("traverseVoltage", traverseMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("traverseCurr", traverseMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("desiredTraversePos", desiredTraversePos.getDegrees());
        SmartDashboard.putNumber("traverseClosedLoopError", traverseMotor.getClosedLoopError().getValueAsDouble());
    }
}

