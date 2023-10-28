// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants.ArmConstants;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;

public class Arm extends SubsystemBase {
  private CANSparkMax pivotMotor = new CANSparkMax(ArmConstants.PIVOT_MOTOR_CAN_ID, MotorType.kBrushless);
  private RelativeEncoder pivotEncoder = pivotMotor.getEncoder();
  //private SparkMaxAbsoluteEncoder pivotAbsoluteEncoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
  private SparkMaxPIDController pivotPIDController = pivotMotor.getPIDController();
  private ArmFeedforward armFeedforward = new ArmFeedforward(ArmConstants.KS, ArmConstants.KG, ArmConstants.KV);
  double setpoint;
  /** Creates a new Arm. */
  public Arm() {
    pivotMotor.setInverted(true);

    pivotEncoder.setPositionConversionFactor(ArmConstants.POSITION_CONVERSION_FACTOR);
    pivotEncoder.setVelocityConversionFactor(ArmConstants.VELOCITY_CONVERSION_FACTOR);
    pivotEncoder.setPosition(0);

    //pivotAbsoluteEncoder.setVelocityConversionFactor(ArmConstants.VELOCITY_CONVERSION_FACTOR);
    //pivotAbsoluteEncoder.setZeroOffset(ArmConstants.OFFSET);

    pivotPIDController.setP(ArmConstants.KP);
    pivotPIDController.setI(ArmConstants.KI);
    pivotPIDController.setD(ArmConstants.KD);

    pivotPIDController.setFeedbackDevice(pivotEncoder);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Pivot Angle", getPivotAngle());
    SmartDashboard.putNumber("Setpoint", setpoint);
  }

  public void setMotorSpeed(double speed){
    pivotMotor.set(speed);
  }
  public double getPivotAngle(){
    return pivotEncoder.getPosition();
    //////////return pivotAbsoluteEncoder.getPosition();
  }
  public void setPivotAngle(double angle){
    setpoint = angle;
    pivotPIDController.setReference(setpoint, CANSparkMax.ControlType.kPosition,0,armFeedforward.calculate(angle*(Math.PI/80), 0));
  }
  public boolean isAtSetpoint(){
    return Math.abs(setpoint-getPivotAngle()) > ArmConstants.SETPOINT_TOLERANCE;
  }
}
