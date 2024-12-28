package frc.team7520.robot.auto;


import edu.wpi.first.wpilibj2.command.*;
import frc.team7520.robot.Constants.ShooterConstants;
import frc.team7520.robot.subsystems.SensorSubsystem;
import frc.team7520.robot.subsystems.shooter.ShooterSubsystem;

public class ShootSequence extends SequentialCommandGroup {

    public ShootSequence() {

        // TODO: Add your sequential commands in the super() call, e.g.
        //           super(new OpenClawCommand(), new MoveArmCommand());
        super(
                new ParallelRaceGroup(
                    new AutoFeeder(0.9, 1),
                    new ParallelCommandGroup(
                        new InstantCommand().until(() -> !SensorSubsystem.getInstance().getColorSensorProximity()),
                        new WaitCommand(0.75)
                    )
                )
        );
    }
}
