// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class TeleopSwerve extends Command {
  private SwerveSubsystem m_SwerveSubsystem;
  private DoubleSupplier m_translationSupplier;
  private DoubleSupplier m_strafeSupplier;
  private DoubleSupplier m_rotationSupplier;
  private BooleanSupplier m_robotCentricSupplier;

  private SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0); //can only change by 3 m/s in the span of 1 s
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);

  private double preRotationVal;
  private Rotation2d rotationTarget;
  /** Creates a new TeleopSwerve. */
  public TeleopSwerve(SwerveSubsystem SwerveSubsystem,
      DoubleSupplier translationSupplier,
      DoubleSupplier strafeSupplier,
      DoubleSupplier rotationSupplier,
      BooleanSupplier robotCentricSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_SwerveSubsystem = SwerveSubsystem;
    addRequirements(m_SwerveSubsystem);
    this.m_translationSupplier = translationSupplier;
    this.m_strafeSupplier = strafeSupplier;
    this.m_rotationSupplier = rotationSupplier;
    this.m_robotCentricSupplier = robotCentricSupplier;

    preRotationVal = 1;
    rotationTarget = Rotation2d.fromDegrees(0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        /* Get Values, applies Deadband, (doesnt do anything if stick is less than a value)*/
    double translationVal =
        translationLimiter.calculate(
            MathUtil.applyDeadband(m_translationSupplier.getAsDouble(), Constants.SwerveConstants.inputDeadband));
    double strafeVal =
        strafeLimiter.calculate(
            MathUtil.applyDeadband(m_strafeSupplier.getAsDouble(), Constants.SwerveConstants.inputDeadband));
    double rotationVal =
        rotationLimiter.calculate(
            MathUtil.applyDeadband(m_rotationSupplier.getAsDouble(), Constants.SwerveConstants.inputDeadband));

    boolean isRotating = Math.abs(rotationVal) > 0.03;
    boolean wasRotating = Math.abs(preRotationVal) > 0.03;
    preRotationVal = rotationVal;
    if (wasRotating && !isRotating){
      rotationTarget = m_SwerveSubsystem.getYaw();
    }
    Rotation2d rotationCurrent = m_SwerveSubsystem.getYaw();
    Rotation2d rotationError = rotationTarget.minus(rotationCurrent);
    
    double angleTarget = clampAngle(rotationTarget.getDegrees());
    double angleCurrent = clampAngle(rotationCurrent.getDegrees());
    double angleError = clampAngleError(angleTarget - angleCurrent);
    angleError = rotationError.getDegrees();
    

    double gyroLock = angleError * 0.005;

    double rotationCommand = 0;
    if (isRotating){
        rotationCommand = rotationVal * Constants.SwerveConstants.maxAngularVelocity;
    }else{
        rotationCommand = gyroLock * Constants.SwerveConstants.maxAngularVelocity;
    }

    /* Drive */
    m_SwerveSubsystem.drive(
        //the joystick values (-1 to 1) multiplied by the max speed of the drivetrain
        new Translation2d(translationVal, strafeVal).times(Constants.SwerveConstants.maxSpeed),
        //rotation value times max spin speed
        rotationCommand,
        /*rotationVal * Constants.SwerveConstants.maxAngularVelocity */
        //whether or not in field centric mode
        true,
        /*!m_robotCentricSupplier.getAsBoolean()*/
        //open loop control
        true);

  }


  public double clampAngle(double angle)
  {
      if (angle < 0)
      {
          angle += 360.0;
      }
      else if (angle > 360.0)
      {
          angle -= 360;
      }
      return angle;
  }

  public double clampAngleError(double error)
  {
      if (error > 180.0) error = 360.0-error;
      else if (error < -180) error = error+360;
      return error;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
