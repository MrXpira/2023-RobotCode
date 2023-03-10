package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class EventAutoTest extends SequentialCommandGroup {
    public EventAutoTest(Swerve s_Swerve, Shooter shooter){
        List<PathPlannerTrajectory> example1 = PathPlanner.loadPathGroup("Event Map Test", new PathConstraints(4, 3));
        // This is just an example event map. It would be better to have a constant, global event map
        // in your code that will be used by all path following commands.
        HashMap<String, Command> eventMap = new HashMap<>();
        //eventMap.put("shootHigh", shooter.moveArmToPosition(1500).andThen(shooter.shoot(Constants.ShooterConstants.highGoalVelocityTopMotor, Constants.ShooterConstants.highGoalVelocityBottomMotor)));
        eventMap.put("intake", shooter.intake());
        eventMap.put("shoot", shooter.shoot(Constants.ShooterConstants.highGoalVelocityTopMotor, Constants.ShooterConstants.highGoalVelocityBottomMotor));


        // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want
        // to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
            s_Swerve::getPose, // Pose2d supplier
            s_Swerve::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
            Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
            new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
            new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
            s_Swerve::setModuleStates, // Module states consumer used to output to the drive subsystem
            eventMap,
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            s_Swerve // The drive subsystem. Used to properly set the requirements of path following commands
        );
        autoBuilder.fullAuto(example1).schedule();
    }
}