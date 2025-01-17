package frc.robot.commands.swervedrive.auto;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Auton;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import java.util.HashMap;
import java.util.List;

import org.ejml.dense.row.decomposition.svd.SafeSvd_DDRM;

public class PathBuilder {
  private SwerveAutoBuilder autoBuilder;

  // TODO: Swerve controller command WPI Library - Use PathplannerTrajectory.transformTrajectoryforalliance
  public PathBuilder(SwerveSubsystem drivebase, HashMap<String, Command> eventMap) {

    autoBuilder = 
        new SwerveAutoBuilder(
            drivebase::getPose, 
            drivebase::resetOdometry, 
            Constants.Swerve.swerveKinematics,
            new PIDConstants(Constants.Auton.kPXController, 0, 0),
            new PIDConstants(Constants.Auton.kPThetaController, 0, 0), 
            drivebase::setModuleStates, 
            eventMap, 
            true, 
            drivebase);
            
  }

  public Command getSwerveCommand(List<PathPlannerTrajectory> path) {
    return autoBuilder.fullAuto(path);
  }
}