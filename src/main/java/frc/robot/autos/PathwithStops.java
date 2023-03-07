package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;

import java.util.ArrayList;
import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.StopEvent;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class PathwithStops extends SequentialCommandGroup {
    public PathwithStops(Swerve s_Swerve){

// This will load the file "FullAuto.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
// for every path in the group
ArrayList<PathPlannerTrajectory> pathGroup = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("Test Path", new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));

// This is just an example event map. It would be better to have a constant, global event map
// in your code that will be used by all path following commands.
HashMap<String, Command> eventMap = new HashMap<>();
// eventMap.put("Balance", s_Swerve.balanceRobot());
//eventMap.put("marker1", armSubsystem.setPosition(Constants.ArmConstants.SCORE_IN_HIGH_CONE).andThen(wristSubsystem.setPosition(Constants.WristConstants.SCORE_IN_HIGH_CONE).andThen(new WaitCommand(1.5)).andThen(collectionSubsystem.collectCone()).andThen(new PrintCommand("getName()").andThen(new WaitCommand(.5).andThen(armSubsystem.setPosition(0)).andThen(wristSubsystem.setPosition(0).andThen(collectionSubsystem.stopMotor()))))));
//eventMap.put("intakeDown", armSubsystem.setPosition(Constants.ArmConstants.ACQUIRE_FROM_FLOOR).andThen(wristSubsystem.setPosition(Constants.WristConstants.ACQUIRE_FROM_FLOOR).andThen(collectionSubsystem.collectCube())));//.andThen(new WaitCommand(1.5)).andThen(collectionSubsystem.collectCone()).andThen(new PrintCommand("workplz").andThen(new WaitCommand(.5).andThen(armSubsystem.setPosition(0)).andThen(wristSubsystem.setPosition(0).andThen(collectionSubsystem.stopMotor()))))));
//eventMap.put("home", armSubsystem.setPosition(25000).andThen(wristSubsystem.setPosition(Constants.WristConstants.HOME).andThen(armSubsystem.setPosition(Constants.ArmConstants.HOME))));//.andThen(new WaitCommand(1.5)).andThen(collectionSubsystem.collectCone()).andThen(new PrintCommand("workplz").andThen(new WaitCommand(.5).andThen(armSubsystem.setPosition(0)).andThen(wristSubsystem.setPosition(0).andThen(collectionSubsystem.stopMotor()))))));


// Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
    s_Swerve::getPose, // Pose2d supplier
    s_Swerve::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
    Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
    new PIDConstants(Constants.AutoConstants.kPXController, 0.0, 0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
    new PIDConstants(Constants.AutoConstants.kPThetaController, 0.0, 0), // PID constants to correct for rotation error (used to create the rotation controller)
    s_Swerve::setModuleStates, // Module states consumer used to output to the drive subsystem
    eventMap,
    false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
    s_Swerve // The drive subsystem. Used to properly set the requirements of path following commands
);
    autoBuilder.fullAuto(pathGroup).schedule();
    }
}