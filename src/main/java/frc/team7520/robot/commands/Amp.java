package frc.team7520.robot.commands;


import edu.wpi.first.wpilibj2.command.*;
import frc.team7520.robot.Constants.ShooterConstants;
import frc.team7520.robot.auto.AutoFeeder;
import frc.team7520.robot.auto.AutoShoot;
import frc.team7520.robot.auto.AutoShootPos;
import frc.team7520.robot.subsystems.SensorSubsystem;
import frc.team7520.robot.subsystems.shooter.ShooterSubsystem;

public class Amp extends SequentialCommandGroup {

    public Amp() {

        // TODO: Add your sequential commands in the super() call, e.g.
        //           super(new OpenClawCommand(), new MoveArmCommand());
        super(
                new ParallelRaceGroup(
                    new AutoShootPos(ShooterConstants.Position.AMP),
                    new WaitCommand(0.3)
                ),
                new ParallelRaceGroup(
                    new AutoShoot(0.127),
                    new AutoFeeder(0.9, 1).until(() -> SensorSubsystem.getInstance().getColorSensorProximity() < ShooterConstants.colourSensorSensedProximity)
                ),
                new ParallelRaceGroup(
                    new WaitCommand(0.1),
                    new AutoShootPos(ShooterConstants.Position.REST)
                ),
                ShooterSubsystem.getInstance().stopShooting()
        );
    }
}
