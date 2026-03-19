// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  private static final int TOP_INTAKE_CAN_ID = 14;
  private static final double TOP_CLOCKWISE_SPEED = 1.0;

  private static final int BOTTOM_INTAKE_CAN_ID = 13;
  private static final double BOTTOM_CLOCKWISE_SPEED = -1.0;

  private final SparkFlex topIntakeMotor = new SparkFlex(TOP_INTAKE_CAN_ID, MotorType.kBrushless);
  private final SparkMax bottomIntakeMotor = new SparkMax(BOTTOM_INTAKE_CAN_ID, MotorType.kBrushless);

  public IntakeSubsystem() {}

  public void runBottomClockwise() {
    bottomIntakeMotor.set(BOTTOM_CLOCKWISE_SPEED);
  }

  public void runBottomCounterClockwise() {
    bottomIntakeMotor.set(-BOTTOM_CLOCKWISE_SPEED);
  }

  public void runTopClockwise() {
    topIntakeMotor.set(-TOP_CLOCKWISE_SPEED);
  }

  public void runTopCounterClockwise() {
    topIntakeMotor.set(TOP_CLOCKWISE_SPEED);
  }

  public void runTriggerIntake() {
    runBottomClockwise();
    runTopClockwise();
  }

  public void stopTop() {
    topIntakeMotor.set(0.0);
  }

  public void stopBottom() {
    bottomIntakeMotor.set(0.0);
  }

  public void stopAll() {
    topIntakeMotor.set(0.0);
    bottomIntakeMotor.set(0.0);
  }

  public Command bottomClockwiseCommand() {
    return runEnd(this::runBottomClockwise, this::stopBottom);
  }

  public Command bottomCounterClockwiseCommand() {
    return runEnd(this::runBottomCounterClockwise, this::stopBottom);
  }

  public Command topClockwiseCommand() {
    return runEnd(this::runTopClockwise, this::stopTop);
  }

  public Command triggerIntakeCommand() {
    return runEnd(this::runTriggerIntake, this::stopAll);
  }
}
