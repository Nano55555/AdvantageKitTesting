// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.subsystems.driveIO.driveIOInputs;

public class DriveSubsystem implements driveIO {

  private static final WPI_TalonFX leftFrontMotor= RobotMap.leftFrontDrivePort;
  private static final WPI_TalonFX rightFrontMotor= RobotMap.rightFrontDrivePort;
  private static final WPI_TalonFX leftBackMotor= RobotMap.leftBackDrivePort;
  private static final WPI_TalonFX rightBackMotor= RobotMap.rightBackDrivePort;

  private final Pigeon2 gyro = new Pigeon2(0);

  private static final double In_To_M=.0254;
  private static final int TICKS_PER_REV=2048;
  private static final double Diameter_Inches=5.0;
  private static final double Wheel_Diameter= Diameter_Inches * In_To_M;
  private static final double Wheel_Circumference= Wheel_Diameter * Math.PI;
  private static final double Gear_Ratio=12.75;
  private static final double Ticks_Per_Meter= ( TICKS_PER_REV * Gear_Ratio)/(Wheel_Circumference);
  private static final double Meters_Per_Ticks= 1/Ticks_Per_Meter;
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
  


    leftFrontMotor.follow(leftBackMotor);
    rightFrontMotor.follow(rightBackMotor); 
    leftFrontMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0,1);
    leftFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    leftFrontMotor.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);
    leftFrontMotor.configVelocityMeasurementWindow(16);
    leftFrontMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1,5,10);

    leftFrontMotor.setNeutralMode(NeutralMode.Coast);
    
    leftFrontMotor.setInverted(false);
    rightFrontMotor.setInverted(true);
    leftBackMotor.setInverted(false);
    rightFrontMotor.setInverted(true);
  }
  public void resetEncoders() {
    leftBackMotor.setSelectedSensorPosition(0);
    rightBackMotor.setSelectedSensorPosition(0);
    leftFrontMotor.setSelectedSensorPosition(0);
    rightFrontMotor.setSelectedSensorPosition(0);
  }
  public void drive(double leftVolts, double rightVolts) {
    leftBackMotor.set(ControlMode.PercentOutput, leftVolts / 12.0);
    rightBackMotor.set(ControlMode.PercentOutput, rightVolts / 12.0);
  }
  public double getRightBackEncoderPosition(){
    return rightBackMotor.getSelectedSensorPosition();
  }
  public double getLeftBackEncoderPosition(){
    return leftBackMotor.getSelectedSensorPosition();
  }
  public double distanceTravelledinTick(){
    return (getLeftBackEncoderPosition() + getRightBackEncoderPosition())/2;
  }
  public double getLeftBackEncoderPositionVelocityMetersPerSecond()
  {
    double leftVelocityMPS = (leftBackMotor.getSelectedSensorPosition()*10);
    leftVelocityMPS = leftVelocityMPS * Meters_Per_Ticks;
    return (leftVelocityMPS);
  }
  public double leftDistanceTravelledInMeters(){
    double left_dist=getRightBackEncoderPosition() * Meters_Per_Ticks;
    return left_dist;
  }
  public void stop() {
    drive(0,0);
  }
    @Override
    public void updateInputs(driveIOInputs inputs){
      inputs.leftPositionRad = Units.rotationsToRadians(
        leftBackMotor.getSelectedSensorPosition() / TICKS_PER_REV / Gear_Ratio);
    inputs.rightPositionRad = Units.rotationsToRadians(
        rightBackMotor.getSelectedSensorPosition() / TICKS_PER_REV / Gear_Ratio);
    inputs.leftVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(
        leftBackMotor.getSelectedSensorVelocity() * 10 / TICKS_PER_REV / Gear_Ratio);
    inputs.rightVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(
        rightBackMotor.getSelectedSensorVelocity() * 10 / TICKS_PER_REV / Gear_Ratio);
        inputs.gyroYawRad = gyro.getYaw();
    }
}
