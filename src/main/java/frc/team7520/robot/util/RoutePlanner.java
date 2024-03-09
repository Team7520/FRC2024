package frc.team7520.robot.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
import frc.team7520.robot.Robot;


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
public class RoutePlanner{
    SendableChooser<Command> m_pathChooser;
    SendableChooser<Command> m_autoChooser;

    public RoutePlanner(SendableChooser<Command> pathChooser, SendableChooser<Command> autoChooser)
    {
        m_pathChooser = pathChooser;
        m_autoChooser = autoChooser;

    }
    public void ConfigureAutoPathProfile()
    {
        // Pickup one game piece from position 1 route
        /*
        new PathPlannerAuto("AutoTest");
        new PathPlannerAuto("BlueShotOnSite");
        new PathPlannerAuto("BlueShotOnSiteLeft");
        new PathPlannerAuto("BlueShotOnSiteRight");
        new PathPlannerAuto("RedShotOnSite");
         */
        new PathPlannerAuto("MiddleSideMove");
        new PathPlannerAuto("RedLeftSideTurn");
        new PathPlannerAuto("RedRightSideTurn");
        new PathPlannerAuto("BlueLeftSideTurn");
        new PathPlannerAuto("BlueRightSideTurn");

    }
    public void ConfigureManualPathProfile()
    {
        // For test and validation purpose.
        // Robot has to path the following test before it could proceed any path planner task
        m_pathChooser.setDefaultOption("X+1m",AutoBuilder.followPath(PathPlannerPath.fromPathFile("X+1m")));
        m_pathChooser.addOption("X-1m", AutoBuilder.followPath(PathPlannerPath.fromPathFile("X-1m")));
        m_pathChooser.addOption("Y+1m", AutoBuilder.followPath(PathPlannerPath.fromPathFile("Y+1m")));
        m_pathChooser.addOption("Y-1m", AutoBuilder.followPath(PathPlannerPath.fromPathFile("Y-1m")));
        m_pathChooser.addOption("XY+1m", AutoBuilder.followPath(PathPlannerPath.fromPathFile("XY+1m")));
        m_pathChooser.addOption("Turn+90", AutoBuilder.followPath(PathPlannerPath.fromPathFile("Turn+90")));
        m_pathChooser.addOption("Turn-90", AutoBuilder.followPath(PathPlannerPath.fromPathFile("Turn-90")));
        m_pathChooser.addOption("DriveCircle", AutoBuilder.followPath(PathPlannerPath.fromPathFile("DriveCircle")));
    }
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */

    public void runPath()
    {
      Command path = getPathPlanerRoute();
      if(path != null)
      {
        path.schedule();
      }
    }

    public Command getPathPlanerRoute()
    {
        return m_pathChooser.getSelected();
    }
}