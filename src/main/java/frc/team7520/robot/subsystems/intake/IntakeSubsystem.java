// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team7520.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXPIDSetConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team7520.robot.Constants;
import frc.team7520.robot.Constants.IntakeConstants;
import frc.team7520.robot.Constants.ShooterConstants;

public class IntakeSubsystem extends SubsystemBase {

    public TalonFX TopMotor = new TalonFX(IntakeConstants.WheelConstants.IntakeTopID);
    public TalonFX BotMotor = new TalonFX(IntakeConstants.WheelConstants.IntakeBottomID);
    public TalonFX RingOneMotor = new TalonFX(IntakeConstants.WheelConstants.RingOneID);
    public TalonFX RingTwoMotor = new TalonFX(IntakeConstants.WheelConstants.RingTwoID);
    public TalonFX FeederMotor = new TalonFX(IntakeConstants.WheelConstants.FeederID);
    
    // private final DigitalInput input = new DigitalInput(0);

    private final SlewRateLimiter slewRateLimiter = new SlewRateLimiter(5);

//    public DiffEncoder diffedEncoder = new DiffEncoder(pivotAbsEncoder, pivotAbsEncoder);

    private final static IntakeSubsystem INSTANCE = new IntakeSubsystem();

    @SuppressWarnings("WeakerAccess")
    public static IntakeSubsystem getInstance() {
        return INSTANCE;
    }

    /** Creates a new ExampleSubsystem. */
    private IntakeSubsystem() {
        BotMotor.setInverted(true);
        RingOneMotor.setInverted(false);
        FeederMotor.setInverted(true);
    }


    public void stop() {
        setSpeed(0);

    }

    public void setSpeed(double speed) {
        setSpeed(speed, false);
    }

    public void setSpeed(double speed, boolean closedLoop) {
        TopMotor.set(speed);
        BotMotor.set(speed);
    }

    public void setRingSpeed(double speed){
        RingOneMotor.set(speed);
        RingTwoMotor.set(speed);
    }

    public void setFeederSpeed(double speed){
        FeederMotor.set(speed);
    }

    // public boolean getSwitchVal() {
    //     return input.get();
    // }

}
