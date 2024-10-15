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
                    new AutoShoot(1),
                    new WaitCommand(0.5)
                ),
                new ParallelCommandGroup(
                    new WaitCommand(1),
                    new AutoFeeder(0.9, 1).until(() -> !SensorSubsystem.getInstance().getColorSensorProximity())
                ),
                ShooterSubsystem.getInstance().stopShooting()
        );
    }
}
