// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team7520.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.team7520.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.team7520.robot.commands.AbsoluteDrive;
import frc.team7520.robot.commands.IntakeStop;
import frc.team7520.robot.commands.ShooterCommand;
import frc.team7520.robot.commands.TeleopDrive;
import frc.team7520.robot.subsystems.Intake.IntakeRollers;
import frc.team7520.robot.subsystems.Intake.IntakePivot;
import frc.team7520.robot.subsystems.Shooter.Shooter;
import frc.team7520.robot.subsystems.swerve.SwerveSubsystem;

import java.io.File;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
    // Subsystems
    private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
            "swerve/neo"));

    private final IntakeRollers IntakeRollersSubsystem = IntakeRollers.getInstance();
    private final IntakePivot IntakeTurningSubsystem = IntakePivot.getInstance();

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final XboxController driverController =
            new XboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
    private final XboxController operatorController = 
            new XboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);

    SequentialCommandGroup autoShoot = new SequentialCommandGroup(Shooter.getInstance().autoShoot().withTimeout(2));

    JoystickButton Shoot = new JoystickButton(operatorController, XboxController.Button.kY.value);
    JoystickButton Amp = new JoystickButton(operatorController, XboxController.Button.kX.value);
    JoystickButton Floor = new JoystickButton(operatorController, XboxController.Button.kA.value);

    JoystickButton Intake = new JoystickButton(operatorController, XboxController.Button.kLeftBumper.value);
    JoystickButton ControlledShooting = new JoystickButton(operatorController, XboxController.Button.kRightBumper.value);
    JoystickButton AmpShoot = new JoystickButton(operatorController, XboxController.Button.kB.value);

    IntakeStop IntakeStop = new IntakeStop(IntakeRollersSubsystem);



    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer()
    {
        // Configure the trigger bindings
        configureBindings();

        // Left joystick is the angle of the robot
        AbsoluteDrive closedAbsoluteDrive = new AbsoluteDrive(drivebase,
                // Applies deadbands and inverts controls because joysticks
                // are back-right positive while robot
                // controls are front-left positive
                () -> MathUtil.applyDeadband(-driverController.getLeftY(),
                        OperatorConstants.LEFT_Y_DEADBAND),
                () -> MathUtil.applyDeadband(-driverController.getLeftX(),
                        OperatorConstants.LEFT_X_DEADBAND),
                () -> -driverController.getRightX(),
                () -> -driverController.getRightY(),
                driverController::getRightBumper,
                driverController::getLeftBumper
        );
        final ShooterCommand shooterCommand = new ShooterCommand(Shooter.getInstance(), operatorController);;
        // Old drive method
        // like in video games
        // Easier to learn, harder to control
        // Not tested not used
        TeleopDrive simClosedFieldRel = new TeleopDrive(drivebase,
                () -> MathUtil.applyDeadband(driverController.getLeftY(),
                        OperatorConstants.LEFT_Y_DEADBAND),
                () -> MathUtil.applyDeadband(driverController.getLeftX(),
                        OperatorConstants.LEFT_X_DEADBAND),
                () -> driverController.getRawAxis(2), () -> true);

        drivebase.setDefaultCommand(closedAbsoluteDrive);
        Shooter.getInstance().setDefaultCommand(shooterCommand);
        IntakeRollersSubsystem.setDefaultCommand(IntakeStop);
    }

    /**
     * Use this method to define named commands for use in {@link PathPlannerAuto}
     *
     */
    private void registerNamedCommands()
    {
        // Example
        NamedCommands.registerCommand("Shoot", new WaitCommand(1));
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings()
    {
        // Zero gyro
        new JoystickButton(driverController, XboxController.Button.kA.value)
                .onTrue(new InstantCommand(drivebase::zeroGyro));
        // X/Lock wheels
        new JoystickButton(driverController, XboxController.Button.kX.value)
                .whileTrue(new InstantCommand(drivebase::lock));
        
        Intake.whileTrue(IntakeRollersSubsystem.Intake());
        ControlledShooting.whileTrue(IntakeRollersSubsystem.ControlledShooting(
                  () -> operatorController.getRawAxis(XboxController.Axis.kRightTrigger.value)
        ));
        AmpShoot.whileTrue(IntakeRollersSubsystem.Amp());
            
        IntakeTurningSubsystem.setDefaultCommand(IntakeTurningSubsystem.Manual(
                  () -> operatorController.getRawAxis(XboxController.Axis.kLeftY.value)
        ));
            
        Shoot.whileTrue(IntakeTurningSubsystem.Shoot());
        Amp.whileTrue(IntakeTurningSubsystem.Amp());
        Floor.whileTrue(IntakeTurningSubsystem.Intake());
    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        return drivebase.getPPAutoCommand("Demo1", true);
    }
}
