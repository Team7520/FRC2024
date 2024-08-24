package frc.team7520.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team7520.robot.Constants;
import frc.team7520.robot.Constants.Swerve;
import frc.team7520.robot.subsystems.intake.IntakeSubsystem;
import frc.team7520.robot.subsystems.swerve.SwerveSubsystem;


public class PathOTF extends Command {
    SwerveSubsystem drivebase;
    int mode;
    Pose2d destination;
    Rotation2d startBezier;
    Rotation2d endBezier;

    public PathOTF(SwerveSubsystem drivebase, int mode, Pose2d destination, Rotation2d startBezier, Rotation2d endBezier) {
        this.drivebase = drivebase;
    }

    @Override
    public void initialize() {
        SwerveSubsystem.pathActive = true;
        var cmd = AutoBuilder.followPath(drivebase.sophisticatedOTFPath(mode, destination, startBezier, endBezier));
        cmd.schedule();
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return SwerveSubsystem.pathActive;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
