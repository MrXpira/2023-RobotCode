package frc.robot.commands.swervedrive.drivebase;

import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;


public class HeadingCorrectionDrive extends CommandBase {    
    private SwerveSubsystem s_Swerve;    
    private double heading;
    private PIDController headignController;

    public HeadingCorrectionDrive(SwerveSubsystem s_Swerve, double heading) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
        this.heading = heading;
        headignController = new PIDController(.05, 0, 0);
    }

    @Override
    public void execute() {
        
        /* Drive */
        s_Swerve.drive(new Translation2d(3,0), headignController.calculate(s_Swerve.getYaw().getDegrees(), heading), false, false);
    }
}