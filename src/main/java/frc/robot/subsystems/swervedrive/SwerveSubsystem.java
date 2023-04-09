package frc.robot.subsystems.swervedrive;

import frc.robot.SwerveModule;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;


public class SwerveSubsystem extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    private final Field2d m_fieldSim = new Field2d();

    

    /*
     * Here we use DifferentialDrivePoseEstimator so that we can fuse odometry
     * readings. The
     * numbers used below are robot specific, and should be tuned.
     */
    
    private PIDController forwardController;


    public SwerveSubsystem() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID, Constants.CANBUS);
        gyro.configFactoryDefault();
        zeroGyro();        
        gyro.setYaw(180);
        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };


        /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();

        forwardController = new PIDController(.048, 0.0001, 0.01);
        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());


    }

    // TODO: UPDATE POSE ESTIMATOR TO WORK WITH LIMELIUGHT
    /** Updates the field-relative position. */
    // public void updateOdometry() {
    //     m_poseEstimator.update(
    //             getYaw(), getModulePositions());

    //     Optional<EstimatedRobotPose> result =
    //             pcw.getEstimatedGlobalPose(m_poseEstimator.getEstimatedPosition());

    //     if (result.isPresent()) {
    //         EstimatedRobotPose camPose = result.get();
    //         m_poseEstimator.addVisionMeasurement(
    //                 camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
    //         m_fieldSim.getObject("Cam Est Pos").setPose(camPose.estimatedPose.toPose2d());
    //     } else {
    //         // move it way off the screen to make it disappear
    //         m_fieldSim.getObject("Cam Est Pos").setPose(new Pose2d(-100, -100, new Rotation2d()));
    //     }

    //     m_fieldSim.setRobotPose(m_poseEstimator.getEstimatedPosition());
    // }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
        
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro(){
        gyro.setYaw(0);
    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    public double getPitch() {
        return gyro.getPitch();
    }


    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    @Override
    public void periodic(){
        swerveOdometry.update(getYaw(), getModulePositions());  

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }

        SmartDashboard.putNumber("Pitch", getPitch());
        SmartDashboard.putData("Field", m_fieldSim);

        m_fieldSim.setRobotPose(getPose());
    }

    public Command lockWheels() {
        return this.run(() -> {
            System.out.println("Locking Wheels");
            SwerveModuleState[] desiredState = {
                new SwerveModuleState(0, new Rotation2d(Math.PI / 4)),
                new SwerveModuleState(0, new Rotation2d(-Math.PI / 4)),
                new SwerveModuleState(0, new Rotation2d(-Math.PI / 4)),
                new SwerveModuleState(0, new Rotation2d(Math.PI / 4))
            };
            setModuleStates(desiredState);
        });
    }  

    public Command balanceRobot() {
        return this.run(() -> {
            System.out.println("Balancing");
            System.out.println(getPitch());
            drive(new Translation2d(-forwardController.calculate(getPitch(),0),0),0, false, true);
        }).until(() -> getPitch() < .1 && getPitch() > -.1);
    }

    public Command revbalanceRobot() {
        return this.run(() -> {
            System.out.println("Balancing");
            System.out.println(getPitch());
            drive(new Translation2d(-forwardController.calculate(getPitch(),0),0),0, false, true);
        }).until(() -> getPitch() < .1 && getPitch() > -.1);
    }

    public Command moveOntoChargeStation() {
        return this.run(() -> {
            System.out.println("Not on platform. Moving forward.");
            drive(new Translation2d(-3,0),0, true, true);
            System.out.println(getPitch());
        }).until(() -> getPitch() > 10.5 || getPitch() < -10.5);
    }

    public Command moveRevOntoChargeStation() {
        return this.run(() -> {
            System.out.println("Not on platform. Moving forward.");
            drive(new Translation2d(3,0),0, true, true);
            System.out.println(getPitch());
        }).until(() -> getPitch() > 10.5 || getPitch() < -10.5);
    }

    public Command moveTimed(double seconds) {
        return this.run(() -> {
            System.out.println("Moving for " + seconds + " seconds");
            drive(new Translation2d(-1.3,0),0, true, true);
            System.out.println(getPitch());
        }).andThen(() -> System.out.println("finished moving")).andThen(() -> drive(new Translation2d(), 0, true, true));
    }

    public Command move( ) {
        return this.run(() -> {
            drive(new Translation2d(-1.3,0),0, true, true);
        });
    }

    public Command moveSlow( ) {
        return this.run(() -> {
            drive(new Translation2d(-.65,0),0, true, true);
        });
    }

    public Command stop( ) {
        return this.runOnce(() -> {
            drive(new Translation2d(0,0),0, true, true);
        });
    }

    public Command balanceRobot(double originalAngle) {
        System.out.println("Balancing");
        return this.run(() -> moveOntoChargeStation())
            .andThen(() -> {
                System.out.println(getPitch());
                drive(new Translation2d(forwardController.calculate(getPitch(),0),0),0, true, true);
            }
        )
        .until(() -> getPitch() < .1 && getPitch() > -.1)
        .andThen(lockWheels());
    }

    // public Command turnTowardsTarget() {
    //     final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
    //     final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);
    //     // Angle between horizontal and the camera.
    //     final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);

    //     // How far from the target we want to be
    //     final double GOAL_RANGE_METERS = Units.feetToMeters(3);

    //     // Change this to match the name of your camera
    //     PhotonCamera camera = new PhotonCamera("photonvision");


    //     // PID constants should be tuned per robot
    //     final double LINEAR_P = 0.1;
    //     final double LINEAR_D = 0.0;
    //     PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);

    //     final double ANGULAR_P = 0.1;
    //     final double ANGULAR_D = 0.0;
    //     PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);
          
    //     return this.run(() -> {
            
    //         double forwardSpeed;
    //         double rotationSpeed;

    //         var result = camera.getLatestResult();

    //         if (result.hasTargets()) {
    //         // First calculate range
    //         double range =
    //                 PhotonUtils.calculateDistanceToTargetMeters(
    //                         CAMERA_HEIGHT_METERS,
    //                         TARGET_HEIGHT_METERS,
    //                         CAMERA_PITCH_RADIANS,
    //                         Units.degreesToRadians(result.getBestTarget().getPitch()));

    //         // Use this range as the measurement we give to the PID controller.
    //         // -1.0 required to ensure positive PID controller effort _increases_ range
    //         forwardSpeed = -forwardController.calculate(range, GOAL_RANGE_METERS);

    //         // Also calculate angular power
    //         // -1.0 required to ensure positive PID controller effort _increases_ yaw
    //         rotationSpeed = -turnController.calculate(result.getBestTarget().getYaw(), 0);
    //         } else {
    //             // If we have no targets, stay still.
    //             forwardSpeed = 0;
    //             rotationSpeed = 0;
    //         }

    //         drive(new Translation2d(forwardSpeed,0),rotationSpeed, false, true);
    //     });
    // }

    public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> {
                if (isFirstPath) {
                    this.resetOdometry(traj.getInitialHolonomicPose());
                }
            }),
            new PPSwerveControllerCommand(
                traj,
                this::getPose,
                Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
                new PIDController(0, 0.0, 0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
                new PIDController(0, 0.0, 0), // PID constants to correct for rotation error (used to create the rotation controller)
                new PIDController(0,0,0),
                this::setModuleStates, // Module states consumer used to output to the drive subsystem
                true,     
                this // The drive subsystem. Used to properly set the requirements of path following commands)
            )
        );
    }
}