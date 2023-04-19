package frc.robot.subsystems.swervedrive;

import frc.robot.SwerveModule;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.subsystems.swervedrive.SwerveDriveTelemetry.TelemetryVerbosity;
import frc.lib.util.MultiLinearInterpolator;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix.Util;
import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
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

    // private final SwerveDriveOdometry swerveOdometry;
    private final SwerveDrivePoseEstimator swerveDrivePoseEstimator;

    private final SwerveModule[] mSwerveMods;

    private final Pigeon2 gyro;

    private final Field2d field = new Field2d();

    /**
     * Trustworthiness of the internal model of how motors should be moving Measured in expected standard deviation
     * (meters of position and degrees of rotation)
     */
    public Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.05, 0.05, 0.05);
    /**
     * Trustworthiness of the vision system Measured in expected standard deviation (meters of position and degrees of
     * rotation)
     */
    public Matrix<N3, N1> visionMeasurementStdDevs = VecBuilder.fill(0.85, 0.85, 0.95);
  

    private PIDController balanceController;

    private boolean useVision = true;

    private final MultiLinearInterpolator oneAprilTagLookupTable = new MultiLinearInterpolator(Constants.VisionConstants.ONE_APRIL_TAG_LOOKUP_TABLE);
    // private NetworkTable limelightTable;
    // private NetworkTableEntry botposeEntry;
    // private NetworkTableEntry targetEntry;
    // private NetworkTableEntry targetPoseEntry;

    /**
     * Creates a new Swerve Subsystem
     */
    public SwerveSubsystem() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID, Constants.CANBUS);
        configGyro();
        zeroGyro();
        
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

        balanceController = new PIDController(.048, 0.0001, 0.01);

        // swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());
        
        swerveDrivePoseEstimator = 
            new SwerveDrivePoseEstimator(
                Constants.Swerve.swerveKinematics,
                getYaw(),
                getModulePositions(),
                new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0)),
                stateStdDevs,
                visionMeasurementStdDevs);

        // limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        // botposeEntry = limelightTable.getEntry(DriverStation.getAlliance() == Alliance.Red ? "botpose_wpired" : "botpose_wpiblue");
        // targetEntry = limelightTable.getEntry("tv");
        // targetPoseEntry = limelightTable.getEntry("targetpose_cameraspace");
        
        /* Initilize Telemetry */
        SmartDashboard.putData("Field", field);

        // Initialize Telemetry Taken from YAGSL
        // if (SwerveDriveTelemetry.verbosity.ordinal() >= TelemetryVerbosity.LOW.ordinal())
        // {
        // SmartDashboard.putData("Field", m_fieldSim);
        // }

        // if (SwerveDriveTelemetry.verbosity.ordinal() >= TelemetryVerbosity.HIGH.ordinal())
        // {
        // SwerveDriveTelemetry.maxSpeed = 6;
        // SwerveDriveTelemetry.maxAngularVelocity = 10;
        // SwerveDriveTelemetry.moduleCount = mSwerveMods.length;
        // SwerveDriveTelemetry.sizeFrontBack = 18;
        // SwerveDriveTelemetry.sizeLeftRight = 18;
        // SwerveDriveTelemetry.wheelLocations = new double[SwerveDriveTelemetry.moduleCount * 2];
        // for (SwerveModule module : mSwerveMods)
        // {
        //     SwerveDriveTelemetry.wheelLocations[module.moduleNumber * 2] = 9;
        //     SwerveDriveTelemetry.wheelLocations[(module.moduleNumber * 2) + 1] = 9;
        // }
        // SwerveDriveTelemetry.measuredStates = new double[SwerveDriveTelemetry.moduleCount * 2];
        // SwerveDriveTelemetry.desiredStates = new double[SwerveDriveTelemetry.moduleCount * 2];
        // }

        /*      SwerveDriveTelemetry.maxSpeed = swerveDriveConfiguration.maxSpeed;
      SwerveDriveTelemetry.maxAngularVelocity = swerveController.config.maxAngularVelocity;
      SwerveDriveTelemetry.moduleCount = swerveModules.length;
      SwerveDriveTelemetry.sizeFrontBack = Units.metersToInches(SwerveMath.getSwerveModule(swerveModules, true,
                                                                                           false).moduleLocation.getX() +
                                                                SwerveMath.getSwerveModule(swerveModules,
                                                                                           false,
                                                                                           false).moduleLocation.getX());
      SwerveDriveTelemetry.sizeLeftRight = Units.metersToInches(SwerveMath.getSwerveModule(swerveModules, false,
                                                                                           true).moduleLocation.getY() +
                                                                SwerveMath.getSwerveModule(swerveModules,
                                                                                           false,
                                                                                           false).moduleLocation.getY());
      SwerveDriveTelemetry.wheelLocations = new double[SwerveDriveTelemetry.moduleCount * 2];
      for (SwerveModule module : swerveModules)
      {
        SwerveDriveTelemetry.wheelLocations[module.moduleNumber * 2] = Units.metersToInches(
            module.configuration.moduleLocation.getX());
        SwerveDriveTelemetry.wheelLocations[(module.moduleNumber * 2) + 1] = Units.metersToInches(
            module.configuration.moduleLocation.getY());
      }
      SwerveDriveTelemetry.measuredStates = new double[SwerveDriveTelemetry.moduleCount * 2];
      SwerveDriveTelemetry.desiredStates = new double[SwerveDriveTelemetry.moduleCount * 2]; */
    }

    @Override
    public void periodic() {
        // swerveOdometry.update(getYaw(), getModulePositions());  

        // for(SwerveModule mod : mSwerveMods){
        //     SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
        //     SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
        //     SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        // }

        // SmartDashboard.putNumber("Pitch", getPitch());
        // SmartDashboard.putNumber("Gyro", gyro.getYaw());
        // SmartDashboard.putNumber("Odom Rotation", getPose().getRotation().getDegrees());
        
        // // System.out.println(getYaw().getDegrees() + " degrees");
        // m_fieldSim.setRobotPose(getPose());
        
        updateOdometry();
        updateVisionPose();
    }

    /**
     *  Updates the field-relative position. 
     */
    public void updateOdometry() {
         // Update odometry
        swerveDrivePoseEstimator.update(getYaw(), getModulePositions());

        field.setRobotPose(swerveDrivePoseEstimator.getEstimatedPosition());

        // if (SwerveDriveTelemetry.verbosity.ordinal() >= TelemetryVerbosity.HIGH.ordinal())
        // {
        //     SwerveDriveTelemetry.updateData();
        // }

        // if (true) {
        //     LimelightResults result = LimelightHelpers.getLatestResults(Constants.VisionConstants.limelightName);
        //     double[] llpose = currentVisionPose();
        //     Pose2d currentPose = new Pose2d(new Translation2d(llpose[0], llpose[1]), new Rotation2d(Units.degreesToRadians(llpose[5])));
        //     if (result.targetingResults.valid && llpose[0] > 0.0 && llpose[1] > 0.0) {
        //         Pose2d visionPose = new Pose2d(llpose[0], llpose[1], Rotation2d.fromDegrees(llpose[5]));

        //         if (Math.abs(currentPose.getX() - getPose().getX()) <= Constants.VisionConstants.maxXYError && Math.abs(currentPose.getY() - getPose().getY()) <= Constants.VisionConstants.maxXYError) {
        //             if (DriverStation.isAutonomousEnabled()) {
        //                 DriverStation.reportWarning("Rejecting vision data with excess error", false);
        //             }
        //         } else {
        //             double[] targetPose = LimelightHelpers.getTargetPose_RobotSpace(Constants.VisionConstants.limelightName);
        //             double targetDistance = Math.sqrt(Math.pow(targetPose[0], 2) + Math.pow(targetPose[1], 2) + Math.pow(targetPose[2], 2));
        //             double[] stddev = oneAprilTagLookupTable.getLookupValue(targetDistance);
        //             if (true) {
        //                 SmartDashboard.putNumber("Target distance", targetDistance);
        //             }
        //             swerveDrivePoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(stddev[0], stddev[1], Units.degreesToRadians(stddev[2])));
        //             swerveDrivePoseEstimator.addVisionMeasurement(visionPose, Timer.getFPGATimestamp() - (llpose[6] / 1000.0));
        //         }
        //     }
        // }
    }

    /** 
     * @return limelight data based on alliance 
     */
    private double[] currentVisionPose() {
        return DriverStation.getAlliance() == Alliance.Blue ? LimelightHelpers.getBotPose_wpiBlue("") : LimelightHelpers.getBotPose_wpiRed("");
    }

    private void updateVisionPose() {
        LimelightResults results =  LimelightHelpers.getLatestResults("");     

        if (results.targetingResults.valid) {
            double[] botpose = currentVisionPose();
            // System.out.println(getVisionTrust(botpose));
            double latency = (Timer.getFPGATimestamp() - (botpose[6]/1000.0));
           
            Pose2d currentPose = new Pose2d(new Translation2d(botpose[0], botpose[1]), new Rotation2d(Units.degreesToRadians(botpose[5])));
            double trustWorthiness = 1;        
            
            if (Math.abs(currentPose.getX() - getPose().getX()) <= 1 && Math.abs(currentPose.getY() - getPose().getY()) <= 1) {
                // System.out.println("Good Data");
                double[] targetPose = LimelightHelpers.getTargetPose_RobotSpace(Constants.VisionConstants.limelightName);
                double targetDistance = Math.sqrt(Math.pow(targetPose[0], 2) + Math.pow(targetPose[1], 2) + Math.pow(targetPose[2], 2));
                double[] stddev = oneAprilTagLookupTable.getLookupValue(targetDistance);
                // System.out.println("DistanceFromTarget: " + targetDistance);
                swerveDrivePoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(stddev[0], stddev[1], Units.degreesToRadians(stddev[2])));
                swerveDrivePoseEstimator.addVisionMeasurement(currentPose, latency, visionMeasurementStdDevs);

            } else {
                System.out.println("Cannot add vision data - Pose is out of range");
            }
            // poseEstimator.addVisionMeasurement(currentPose, latency,VecBuilder.fill(0.9, 0.9, 0.1).times(1.0 / trustWorthiness));
        }
        
    }

    // private Matrix<N3, N1> getVisionTrust(double[] botpose) {
    //     Pose2d distanceFromTag = distanceFromTag();
    //     System.out.println("Distance X: " + distanceFromTag.getX());
    //     return VecBuilder.fill(0.1 * distanceFromTag.getX(), 0.1 * distanceFromTag.getY(), 0.95);
    // }

    private Pose2d distanceFromTag() {
        return LimelightHelpers.getBotPose3d_TargetSpace("").toPose2d();
    }

    public void configGyro() {
        gyro.configFactoryDefault();
        gyro.configEnableCompass(false);
        gyro.configDisableNoMotionCalibration(true);
        gyro.configDisableTemperatureCompensation(true);
    }

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
        return swerveDrivePoseEstimator.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        swerveDrivePoseEstimator.resetPosition(pose.getRotation(), getModulePositions(), pose);
        gyro.setYaw(pose.getRotation().getDegrees());
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

    public Command lockWheels() {
        return this.run(() -> {
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
        System.out.println("Balancing");
        return this.run(() -> {
            drive(new Translation2d(balanceController.calculate(getPitch(),0),0),0, false, true);
        }).until(() -> getPitch() < .1 && getPitch() > -.1).andThen(() -> System.out.println("Finished Balacing"));
    }

    public Command revbalanceRobot() {
        return this.run(() -> {
            System.out.println("Balancing");
            System.out.println(getPitch());
            drive(new Translation2d(-balanceController.calculate(getPitch(),0),0),0, false, true);
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

    public Command move() {
        return this.run(() -> {
            drive(new Translation2d(-1.3,0),0, true, true);
        });
    }

    public Command moveSlow() {
        return this.run(() -> {
            drive(new Translation2d(-.65,0),0, true, true);
        });
    }

    public Command stop() {
        return this.runOnce(() -> {
            drive(new Translation2d(0,0),0, true, true);
        });
    }

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
                new PIDController(.5,0,0),
                this::setModuleStates, // Module states consumer used to output to the drive subsystem
                true,     
                this // The drive subsystem. Used to properly set the requirements of path following commands)
            )
        );
    }
}