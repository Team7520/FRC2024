// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team7520.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.team7520.robot.auto.AutoIntake;
// import frc.team7520.robot.Constants.IntakeConstants;
// import frc.team7520.robot.Constants.IntakeConstants.Position;
import frc.team7520.robot.auto.AutoShootPos;
import frc.team7520.robot.auto.AutoShootRest;
import frc.team7520.robot.auto.ShootSequence;
import frc.team7520.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
// import frc.team7520.robot.auto.AutoIntake;
// import frc.team7520.robot.auto.AutoShoot;
// import frc.team7520.robot.auto.ShootSequence;
import frc.team7520.robot.commands.AbsoluteDrive;
// import frc.team7520.robot.commands.Amp;
//import frc.team7520.robot.commands.Climber;
import frc.team7520.robot.commands.Intake;
import frc.team7520.robot.commands.Sensor;
import frc.team7520.robot.commands.Shooter;

import frc.team7520.robot.commands.TeleopDrive;
import frc.team7520.robot.subsystems.LED;
import frc.team7520.robot.subsystems.SensorSubsystem;
import frc.team7520.robot.subsystems.intake.IntakeSubsystem;
import frc.team7520.robot.subsystems.shooter.ShooterSubsystem;
import frc.team7520.robot.subsystems.swerve.SwerveSubsystem;
import frc.team7520.robot.Constants.ShooterConstants;
import frc.team7520.robot.Constants.ShooterConstants.Position;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
    // Subsystems
    public final SwerveSubsystem drivebase = SwerveSubsystem.getInstance();

    private final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();

    private final IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();

    private final SensorSubsystem sensorSubsystem = SensorSubsystem.getInstance();

    private final LED LEDSubsystem = LED.getInstance();

    public final SendableChooser<Command> autoChooser = new SendableChooser<>();

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final XboxController driverController =
            new XboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

    private final XboxController operatorController =
            new XboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);

    private final Intake intake = new Intake(
            intakeSubsystem,
            sensorSubsystem,
            operatorController::getAButton,
            operatorController::getBButton,
            operatorController::getRightTriggerAxis // fire
        );

    private final Sensor sensor = new Sensor(
        sensorSubsystem,
        LEDSubsystem,
        sensorSubsystem::getBeamBreak,
        sensorSubsystem::getColorSensorProximity
        );

    private final Shooter shooter = new Shooter(shooterSubsystem,
        operatorController::getLeftTriggerAxis,
            operatorController::getXButton,
            operatorController::getYButton,
            operatorController::getPOV
        );

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer()
    {

        registerAutos();

        //CameraServer.startAutomaticCapture();

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
                () -> driverController.getRightX(),
                () -> driverController.getRightY(),
                driverController::getRightBumper,
                driverController::getLeftBumper,
                driverController::getBButtonPressed
        );

        // Old drive method
        // like in video games
        // Easier to learn, harder to control
        // Not tested not used
        TeleopDrive teleopDrive = new TeleopDrive(drivebase,
                () -> MathUtil.applyDeadband(driverController.getLeftY(),
                        OperatorConstants.LEFT_Y_DEADBAND),
                () -> MathUtil.applyDeadband(driverController.getLeftX(),
                        OperatorConstants.LEFT_X_DEADBAND),
                () -> MathUtil.applyDeadband(-driverController.getRightX(),
                        OperatorConstants.RIGHT_X_DEADBAND)
                , () -> true);

        drivebase.setDefaultCommand(teleopDrive);
        shooterSubsystem.setDefaultCommand(shooter);
        intakeSubsystem.setDefaultCommand(intake);
        sensorSubsystem.setDefaultCommand(sensor);
    }

    private void registerAutos(){

        registerNamedCommands();


        // TODO: Make real autos with auto mirror
        autoChooser.addOption("Shoot", drivebase.getAutonomousCommand("Shoot"));
        autoChooser.addOption("4NoteAuto(Note1)", drivebase.getAutonomousCommand("4NoteAuto(Note1First)"));
        autoChooser.addOption("4NoteAuto(Note3)", drivebase.getAutonomousCommand("4NoteAuto(Note3First)"));
        autoChooser.addOption("CenterAuto(Source)", drivebase.getAutonomousCommand("CenterAuto(Source)"));
        autoChooser.addOption("CenterAuto(Middle)", drivebase.getAutonomousCommand("CenterAuto(Middle)"));
        autoChooser.addOption("CenterAuto(Amp)", drivebase.getAutonomousCommand("CenterAuto(Amp)"));
        autoChooser.addOption("4NoteNote1Test", drivebase.getAutonomousCommand("4NoteNote1"));
        autoChooser.addOption("4NoteNote3Test", drivebase.getAutonomousCommand("4NoteNote3"));
        autoChooser.addOption("CenterSourceTest", drivebase.getAutonomousCommand("CenterSource"));
        autoChooser.addOption("CenterAmpTest", drivebase.getAutonomousCommand("CenterAmp"));
        autoChooser.addOption("CenterMiddleTest", drivebase.getAutonomousCommand("CenterMiddle"));

        SmartDashboard.putData(autoChooser);
        // SmartDashboard.putBoolean("Shooting", true);
    }

    /**
     * Use this method to define named commands for use in {@link PathPlannerAuto}
     *
     */
    private void registerNamedCommands()
    {
        // Example
        NamedCommands.registerCommand("setShootPosition", new AutoShootPos());
        // NamedCommands.registerCommand("shootSubwooferCenter", new AutoShootPos(Position.SUBWOOFERCENTER));
        // NamedCommands.registerCommand("shootSubwooferRight", new AutoShootPos(Position.SUBWOOFERRIGHT));
        // NamedCommands.registerCommand("shootSubwooferLeft", new AutoShootPos(Position.SUBWOOFERLEFT));
        // NamedCommands.registerCommand("shootNoteCW", new AutoShootPos(Position.PODIUMBLUE));
        // NamedCommands.registerCommand("shootNoteCenter", new AutoShootPos(Position.NOTECENTER));
        // NamedCommands.registerCommand("shootNoteCCW", new AutoShootPos(Position.PODIUMRED));
        // NamedCommands.registerCommand("shootWinglineBlue", new AutoShootPos(Position.WINGLINEBLUE));
        // NamedCommands.registerCommand("shootWinglineRed", new AutoShootPos(Position.WINGLINERED));
        NamedCommands.registerCommand("shooterRest", new AutoShootRest());
        NamedCommands.registerCommand("shoot", new ShootSequence());
        // NamedCommands.registerCommand("log", new InstantCommand(() -> System.out.println("eeeeeeeeeeeeeeeeeeeeeeeee")));
        // NamedCommands.registerCommand("intake", new AutoIntake(0.6, 0.85, 0.2).until(() -> sensorSubsystem.getColorSensorProximity() > ShooterConstants.colourSensorSensedProximity));
        NamedCommands.registerCommand("intake", new AutoIntake(0.6, 0.85, 0.2).raceWith(new WaitCommand(1)));
        // NamedCommands.registerCommand("stopIntake", new InstantCommand(() -> new AutoIntake(0, 0, 0)));
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
                .whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock)));

//        new JoystickButton(operatorController, XboxController.Button.kX.value)
//                .onTrue(new Amp());
    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {

        return autoChooser.getSelected();
        // return new SequentialCommandGroup(
        //         new ParallelCommandGroup(
        //                 new InstantCommand(() -> shooterSubsystem.setDefaultCommand(new AutoShoot(0.7, false))),
        //                 autoChooser.getSelected()
        //         ),
        //         new InstantCommand(() -> shooterSubsystem.setDefaultCommand(shooter))
        // ).finallyDo((boolean inturupted) -> {
        //     if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
        //         drivebase.setGyro(drivebase.getHeading().minus(Rotation2d.fromDegrees(180)));
        //     }

        //     shooterSubsystem.setDefaultCommand(shooter);
        // });
    }

    public void teleOpInit() {

        //shooterSubsystem.setDefaultCommand(shooter);

    }
}
