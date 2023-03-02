package frc.robot.autos;

import frc.robot.subsystems.Swerve;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class exampleAuto extends SequentialCommandGroup {
    public exampleAuto(Swerve s_Swerve){
        PathPlannerTrajectory test = PathPlanner.loadPath("New Path", 3,3);
        s_Swerve.followTrajectoryCommand(test, true);
    }
}