package frc.robot.commands;

import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class ChaseTagCommand extends CommandBase{
    
    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3,2);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3,2);
    private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(8,8);

    private static final int TAG_TO_CHASE = 2;
    private static final Transform3d ROBOT_TO_CAMERA = new Transform3d(new Translation3d(14.5,0,0), new Rotation3d(0,0, Math.PI));
    private static final Transform3d TAG_TO_GOAL = new Transform3d(new Translation3d(1.5,0,0), new Rotation3d(0,0, Math.PI));

    private PhotonCamera photonCamera;
    private Swerve swerve;
    private Supplier<Pose2d> poseProvider;

    private final ProfiledPIDController x_PidController = new ProfiledPIDController(3, 0, 0, X_CONSTRAINTS);
    private final ProfiledPIDController y_PidController =  new ProfiledPIDController(3, 0, 0, Y_CONSTRAINTS);
    private final ProfiledPIDController omega_PidController =  new ProfiledPIDController(2, 0, 0, OMEGA_CONSTRAINTS);
    private PhotonTrackedTarget lastTarget;

    public ChaseTagCommand(
        PhotonCamera photonCamera,
        Swerve swerve,
        Supplier<Pose2d> poseProvider){
        
            this.photonCamera =  photonCamera;
            this.swerve = swerve;
            x_PidController.setTolerance(.2);
            y_PidController.setTolerance(.2);
            omega_PidController.setTolerance(Units.degreesToRadians(3));
            omega_PidController.enableContinuousInput(-Math.PI, Math.PI);

            addRequirements(swerve);
        
    }

    @Override
    public void initialize(){
        lastTarget = null;
        var robotPose = poseProvider.get();
        omega_PidController.reset(robotPose.getRotation().getRadians());
        x_PidController.reset(robotPose.getX());
        y_PidController.reset(robotPose.getY());
    }
    
    @Override 
    public void execute(){
        var robotPose2d = poseProvider.get();
        var robotPose = new Pose3d(robotPose2d.getX(),robotPose2d.getY(),0, new Rotation3d(0,0,robotPose2d.getRotation().getRadians()));

        var photonRes = photonCamera.getLatestResult();
        if(photonRes.hasTargets()){
            var targetOpt = photonRes.getTargets().stream()
                .filter(t -> t.getFiducialId() == TAG_TO_CHASE)
                .filter(t -> !t.equals(lastTarget) && t.getPoseAmbiguity() <= .2 && t.getPoseAmbiguity() >= 0)
                .findFirst();
            if(targetOpt.isPresent()){
                var target = targetOpt.get();

                lastTarget = target;

                var cameraPose = robotPose.transformBy(ROBOT_TO_CAMERA);

                var camToTarget = target.getBestCameraToTarget();

                var targetPose = cameraPose.transformBy(camToTarget);

                var goalPose = targetPose.transformBy(TAG_TO_GOAL).toPose2d();

                x_PidController.setGoal(goalPose.getX());
                y_PidController.setGoal(goalPose.getY());
                omega_PidController.setGoal(goalPose.getRotation().getRadians());
            }
        }

        


    }
    
}
