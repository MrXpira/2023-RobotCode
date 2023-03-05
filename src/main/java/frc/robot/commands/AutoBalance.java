// package frc.robot.commands;

// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.Swerve;


// // This command self=balances on the charging station using gyroscope pitch as feedback
// public class AutoBalance extends CommandBase {

//   private Swerve swerve;

//   private double error;
//   private double currentAngle;
//   private double drivePower;

//   public static final double BEAM_BALANACED_DRIVE_KP = .01; // P (Proportional) constant of a PID
//   // loop
//   public static final double BEAM_BALANCED_GOAL_DEGREES = 0;
//   public static final double BEAM_BALANCED_ANGLE_TRESHOLD_DEGREES = 1;
//   public static final double BACKWARDS_BALANCING_EXTRA_POWER_MULTIPLIER = 1.5;

//   /** Command to use Gyro data to resist the tip angle from the beam - to stabalize and balanace */
//   public AutoBalance(Swerve swerve) {
//     this.swerve = swerve;
//     addRequirements(swerve);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     // Uncomment the line below this to simulate the gyroscope axis with a controller joystick
//     // Double currentAngle = -1 * Robot.controller.getRawAxis(Constants.LEFT_VERTICAL_JOYSTICK_AXIS)
//     // * 45;
//     this.currentAngle = swerve.getPitch();

//     error = BEAM_BALANCED_GOAL_DEGREES - currentAngle;
//     drivePower = -Math.min(BEAM_BALANACED_DRIVE_KP * error, 1);

//     // Our robot needed an extra push to drive up in reverse, probably due to weight imbalances
//     if (drivePower < 0) {
//       drivePower *= BACKWARDS_BALANCING_EXTRA_POWER_MULTIPLIER;
//     }

//     // Limit the max power
//     if (Math.abs(drivePower) > 0.4) {
//       drivePower = Math.copySign(0.8, drivePower);
//     }

//     swerve.runVelocity(new ChassisSpeeds(drivePower, 0, 0));

//     // Debugging Print Statments
//     System.out.println("Current Angle: " + currentAngle);
//     System.out.println("Error " + error);
//     System.out.println("Drive Power: " + drivePower);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     swerve.stop();
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return Math.abs(error) < BEAM_BALANCED_ANGLE_TRESHOLD_DEGREES; // End the command when we are
//     // within the specified threshold
//     // of being 'flat' (gyroscope
//     // pitch of 0 degrees)
//   }
// }