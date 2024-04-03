package frc.team7520.robot.auto;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.team7520.robot.Constants;
import frc.team7520.robot.subsystems.intake.IntakeSubsystem;
import frc.team7520.robot.subsystems.shooter.ShooterSubsystem;

public class ShootSequence extends SequentialCommandGroup {
        static int shootCount = 1;

    public ShootSequence() {

        // TODO: Add your sequential commands in the super() call, e.g.
        //           super(new OpenClawCommand(), new MoveArmCommand());
        super(
                new ParallelCommandGroup(
                        new AutoIntake(Constants.IntakeConstants.Position.SHOOT),
                        new ParallelRaceGroup(
                                new AutoShoot(1, false),
                                new WaitCommand(0.25)
                        )
                ),
                new InstantCommand(() -> IntakeSubsystem.getInstance().setSpeed(1)),
                new WaitCommand(0.3),
                new InstantCommand(() -> IntakeSubsystem.getInstance().setSpeed(0))
//                new InstantCommand(() -> ShooterSubsystem.getInstance().setSpeed(0, false))
        );
    }

    public ShootSequence(double power) {

        // TODO: Add your sequential commands in the super() call, e.g.
        //           super(new OpenClawCommand(), new MoveArmCommand());
        super(
                new InstantCommand(()->{SmartDashboard.putNumber("Shoot Count", shootCount++);}),
                new ParallelCommandGroup(
                        new AutoIntake(Constants.IntakeConstants.Position.SHOOT),
                        new ParallelRaceGroup(
                                new AutoShoot(power, false),
                                new WaitCommand(0.5)
                        )
                ),                
                new InstantCommand(() -> IntakeSubsystem.getInstance().setSpeed(1)),
                new WaitCommand(1.5),
                new InstantCommand(() -> IntakeSubsystem.getInstance().setSpeed(0))
//                new InstantCommand(() -> ShooterSubsystem.getInstance().setSpeed(0, false))
        );
    }
}
