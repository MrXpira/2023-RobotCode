package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix.CANifier.PinValues;
import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;


public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;

    private PIDController forwardController;
    private PIDController pitchController;

    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.configFactoryDefault();
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

        forwardController = new PIDController(.02355, 0, 0.01);
        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());
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
    }

    public Command lockWheels() {
        return this.run(() -> {
            SwerveModuleState[] desiredState = {
                new SwerveModuleState(0.0,new Rotation2d(-45)),
                new SwerveModuleState(0.0,new Rotation2d(45)),
                new SwerveModuleState(0.0,new Rotation2d(45)),
                new SwerveModuleState(0.0,new Rotation2d(-45))
            };
            setModuleStates(desiredState);
        });
    }  

    public Command balanceRobot() {
        return this.run(() -> {
            System.out.println("Balancing");
            System.out.println(getPitch());
            drive(new Translation2d(forwardController.calculate(getPitch(),0),0),0, false, true);
        }).until(() -> getPitch() < .1 && getPitch() > -.1);
    }

    public Command moveOntoChargeStation() {
        
        return this.run(() -> {
            System.out.println("Not on platform. Moving forward.");
            drive(new Translation2d(.5,0),0, false, true);
            System.out.println(getPitch());
        }).until(() -> getPitch() > 5 || getPitch() > 5);
    }
    public Command balanceRobot(double originalAngle) {
        System.out.println("Balancing");
        return this.run(() -> moveOntoChargeStation())
            .andThen(() -> {
                System.out.println(getPitch());
                drive(new Translation2d(forwardController.calculate(getPitch(),0),0),0, false, true);
            }
        )
        .until(() -> getPitch() < .1 && getPitch() > -.1)
        .andThen(lockWheels());
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
                new PIDController(Constants.AutoConstants.kPXController, 0.0, 0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
                new PIDController(Constants.AutoConstants.kPYController, 0.0, 0), // PID constants to correct for rotation error (used to create the rotation controller)
                new PIDController(Constants.AutoConstants.kPThetaController,0,0),
                this::setModuleStates, // Module states consumer used to output to the drive subsystem
                true,     
                this // The drive subsystem. Used to properly set the requirements of path following commands)
            )
        );
    }
}