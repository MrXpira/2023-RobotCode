package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

import java.util.HashMap;
import java.util.List;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.TreeMap;

public final class Constants {
    public static final double stickDeadband = 0.1;
    public static final int CANdleID = 40;
    public static final String CANBUS = "rio";

    public static final class ArmConstants {
        public static final int ARM_MAIN_MOTOR = 13;
        public static final int ARM_FOLLOWER_MOTOR = 14;
    
        public static final int kTimeoutMs = 0;
        public static final int kPIDLoopIdx = 0;
    
        /* Sysid values divided by 12 to convert from voltage */
        public static final double armkG = (.36);
        public static final double armkS = (0);
        public static final double openLoopRamp = 0.25;
        public static final double motionCruiseVelocity = 7200;
        public static final double motionAcceleration = 6000;
        public static final double kA = 0.02;
        public static final double kV = .040;
        public static final double armkP = 0.1546;//.15;
        public static final double armkI = 0;
        public static final double armkD = 2;//9.44;
        public static final double armkF = .07885; //kV * (1023/12.0);
    
        public static final double kArmEncoderDistPerPulse = 2.0 * Math.PI / 2048;
    
        public static final double shooterArmPeakCurrentDuration = 1;
        public static final double shooterArmPeakCurrentLimit = 55;
        public static final double shooterArmContinuousCurrentLimit = 40;
        public static final boolean shooterArmEnableCurrentLimit = true;
        
    
    
        public static final double MidPosition = 0;
        public static final double restPosition = 0;
        public static final double lowPosition = 0;
        public static final double highPosition = 0;
        public static final double intakePosition = 20000 ;
        public static final double cannonPosition = 7000;
        public static final int ArmGearRatio = 32;
        public static final double lowStackPosition = 16000;
        
      }

    public static final class ShooterConstants {
    public static final int SHOOTER_TOP_MOTOR = 15;
    public static final int SHOOTER_BOTTOM_MOTOR = 16;

    public static final double highGoalVelocityTopMotor = .40;//.36;//.33;//.27;
    public static final double highGoalVelocityBottomMotor = .44;//.37;//.31;

    public static final double midGoalVelocityBottomMotor = .23;
    public static final double midGoalVelocityTopMotor = .23;

    public static final double bottomGoalVelocityTopMotor = .16;
    public static final double bottomGoalVelocityBottomMotor = .08;
    
    public static final double cannonGoalVelocityTopMotor = .70;
    public static final double cannonGoalVelocityBottomMotor = .70;

    public static final double cannon2GoalVelocityTopMotor = .50;
    public static final double cannon2GoalVelocityBottomMotor = .54;


    public static final double intakeVelocity = .3;
    public static final int currentThreshold = 25;
    public static final double shootWaitTime = 1;
    

  }
    public static final class Swerve {
        public static final int pigeonID = 30;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
            COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(18.75); //TODO: This must be tuned to specific robot
        public static final double wheelBase = Units.inchesToMeters(18.75); //TODO: This must be tuned to specific robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
        public static final boolean driveMotorInvert = false; //chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = chosenModule.canCoderInvert;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.0; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double driveKS = (0.32 / 12); //TODO: This must be tuned to specific robot
        public static final double driveKV = (1.51 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.97;
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 3;
            
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(288.545);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 6;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(255.850);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 9;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(356.924);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 10;
            public static final int angleMotorID = 11;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(275.273);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class Auton
  {  
    /* Balance PID Values */
    public static final PIDController balancePID = new PIDController(.048, 0.0001, 0.01);

    /* Pathplanner */
    public static final double MAX_SPEED        = 4;
    public static final double MAX_ACCELERATION = 2;

    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    /* Constraint for the motion profilied robot angle controller */
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
    new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

    // public static final double lineUpMid = 1.73;
    // public static final List<ScoringArea> scoreAreaList =
        // new ArrayList<ScoringArea>() {
        //   {
        //     add(
        //         new ScoringArea(
        //             new RectanglePoseArea(
        //                 new Translation2d(1.23, 3.53), new Translation2d(2.86, 5.33)),
        //             // diagonal y's should not overlap
        //             new HolonomicPose2d(new Pose2d(lineUpMid, 4.95, new Rotation2d(Math.PI)), new Rotation2d()),
        //             new HolonomicPose2d(new Pose2d(lineUpMid, 4.40, new Rotation2d(Math.PI)), new Rotation2d()),
        //             new HolonomicPose2d(
        //                 new Pose2d(lineUpMid, 3.84, new Rotation2d(Math.PI)), new Rotation2d())));
        //     add(
        //         new ScoringArea(
        //             new RectanglePoseArea(
        //                 new Translation2d(1.23, 1.90), new Translation2d(2.92, 3.52)),
        //             new HolonomicPose2d(new Pose2d(lineUpMid, 3.30, new Rotation2d(Math.PI)), new Rotation2d()),
        //             new HolonomicPose2d(new Pose2d(lineUpMid, 2.72, new Rotation2d(Math.PI)), new Rotation2d()),
        //             new HolonomicPose2d(
        //                 new Pose2d(lineUpMid, 2.19, new Rotation2d(Math.PI)), new Rotation2d())));
        //     add(
        //         new ScoringArea(
        //             new RectanglePoseArea(
        //                 new Translation2d(1.23, 0.0), new Translation2d(2.89, 1.89)),
        //             new HolonomicPose2d(new Pose2d(lineUpMid, 1.61, new Rotation2d(Math.PI)), new Rotation2d()),
        //             new HolonomicPose2d(new Pose2d(lineUpMid, 1.03, new Rotation2d(Math.PI)), new Rotation2d()),
        //             new HolonomicPose2d(
        //                 new Pose2d(lineUpMid, 0.55, new Rotation2d(Math.PI)), new Rotation2d())));
        //   }
        // };

    public static final double maxSpeedMPS = 5;

    public static final double maxAccelerationMPS = 3;

	public static final double moveOntoChargeStationSpeed = 3;

    public static final double kPXController = 8;

    public static final double kPThetaController = 1;//.57;
  }

    
}
