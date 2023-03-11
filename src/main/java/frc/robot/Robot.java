// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.cameraserver.CameraServer;



public class Robot extends TimedRobot {
  /*
   * Autonomous selection options.
   */
  private static final String kNoMobilityAuto = "no mobility";
  private static final String kConeAuto = "cone";
  private static final String kCubeAuto = "cube";
  private static final String kConeBalanceAuto = "cone balance";
  private static final String kCubeBalanceAuto = "cube balance";
  private static final String kMobilityOnlyAuto = "mobility only";
  private String m_autoSelected;
  private final SendableChooser<String> m_autoChooser = new SendableChooser<>();

  /*
   * Drive motor controller instances.
   * 
   * Tank drive system using dual Logitech joysticks
   * Grouped to left and right side
   */

  WPI_TalonFX driveLFTalon = new WPI_TalonFX(3);
  WPI_TalonFX driveLRTalon = new WPI_TalonFX(9);
  MotorControllerGroup left = new MotorControllerGroup(driveLFTalon, driveLRTalon);

  WPI_TalonFX driveRFTalon = new WPI_TalonFX(50);
  WPI_TalonFX driveRRTalon = new WPI_TalonFX(5);
  MotorControllerGroup right = new MotorControllerGroup(driveRFTalon, driveRRTalon);

  DifferentialDrive tankDrive = new DifferentialDrive(left, right);

  Pigeon2 tiltSensor = new Pigeon2(4, null);

  double pitch = 0;
  double yaw = 0;

  public double getPitch() {
    return pitch;
  }

  public double getYaw() {
    return yaw;
  }

  /*
   * Mechanism motor controller instances.
   * 
   * Like the drive motors, set the CAN id's to match your robot or use different
   * motor controller classses (TalonFX, TalonSRX, Spark, VictorSP) to match your
   * robot.
   * 
   * The arm is a NEO on Everybud.
   * The intake is a NEO 550 on Everybud.
   */
  WPI_TalonFX arm = new WPI_TalonFX(1);
  CANSparkMax intake = new CANSparkMax(11, MotorType.kBrushless);

  double randomVariable = 0;

  

  /**
   * The starter code uses the most generic joystick class.
   * 
   * The reveal video was filmed using a logitech gamepad set to
   * directinput mode (switch set to D on the bottom). You may want
   * to use the XBoxController class with the gamepad set to XInput
   * mode (switch set to X on the bottom) or a different controller
   * that you feel is more comfortable.
   */

  GenericHID Gpad = new GenericHID(0);
  Joystick JoystickLeft = new Joystick(1);
  Joystick JoystickRight = new Joystick(2);

  AddressableLED ledStrip = new AddressableLED(9);
  AddressableLEDBuffer ledStripBuffer = new AddressableLEDBuffer(15);
  int rainbowFirstPixelHue = 0;
  
  /*
   * Magic numbers. Use these to adjust settings.
   */

  /**
   * How many amps the arm motor can use.
   */
  static final int ARM_CURRENT_LIMIT_A = 20;

  /**
   * Percent output to run the arm up/down at
   */
  static final double ARM_OUTPUT_POWER = 0.3;

  /**
   * How many amps the intake can use while picking up
   */
  static final int INTAKE_CURRENT_LIMIT_A = 25;

  /**
   * How many amps the intake can use while holding
   */
  static final int INTAKE_HOLD_CURRENT_LIMIT_A = 5;

  /**
   * Percent output for intaking
   */
  static final double INTAKE_OUTPUT_POWER = 0.40;

  /**
   * Percent output for holding
   */
  static final double INTAKE_HOLD_POWER = 0.0;

  /**
   * Time to extend or retract arm in auto
   */
  static final double ARM_EXTEND_TIME_S = 2.0;

  /**
   * Time to throw game piece in auto
   */
  static final double AUTO_THROW_TIME_S = 0.375;

  /**
   * Time to drive back in auto
   */
  static final double AUTO_DRIVE_TIME = 6.0;

  /**
   * Speed to drive backwards in auto
   */
  static final double AUTO_DRIVE_SPEED = 0.20;

  /**
   * This method is run once when the robot is first started up.
   */
  @Override
  public void robotInit() {
    m_autoChooser.setDefaultOption("no mobility", kNoMobilityAuto);
    m_autoChooser.addOption("cone and mobility", kConeAuto);
    m_autoChooser.addOption("cube and mobility", kCubeAuto);
    m_autoChooser.addOption("cone and balancing", kConeBalanceAuto);
    m_autoChooser.addOption("cube and balancing", kCubeBalanceAuto);
    m_autoChooser.addOption("mobility only", kMobilityOnlyAuto);
    SmartDashboard.putData("choose autonomous setup", m_autoChooser);
    CameraServer.startAutomaticCapture();


    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    ledStrip.setLength(ledStripBuffer.getLength());

    // Set the data
    ledStrip.setData(ledStripBuffer);
    ledStrip.start();

    /*m_controlsChooser.setDefaultOption("gamepads only", kDualGamepads);
    m_controlsChooser.addOption("joysticks and gamepad", kJoysticksGamepad);
    SmartDashboard.putData("choose control layout", m_controlsChooser);*/

    /*
     * You will need to change some of these from false to true.
     * 
     * In the setDriveMotors method, comment out all but 1 of the 4 ca  ``lls
     * to the set() methods. Push the joystick forward. Reverse the motor
     * if it is going the wrong way. Repeat for the other 3 motors.
     * Issue here: Is this correct?
     */
    driveLFTalon.setInverted(true);
    driveLRTalon.setInverted(true);
    driveRFTalon.setInverted(false);
    driveRRTalon.setInverted(false);

    /*
     * Set the arm and intake to brake mode to help hold position.
     * If either one is reversed, change that here too. Arm out is defined
     * as positive, arm in is negative.
     */

    arm.setInverted(false);
    intake.setInverted(false);
    intake.setIdleMode(IdleMode.kBrake);
  }

  public void yellowStrip() {
    for (var i = 0; i < ledStripBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      ledStripBuffer.setRGB(i, 255, 255, 0);
   }
   
   ledStrip.setData(ledStripBuffer);


  }

  public void purpleStrip() {
    for (var i = 0; i < ledStripBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      ledStripBuffer.setRGB(i, 163, 1, 255);
   }
   
   ledStrip.setData(ledStripBuffer);


  }

  public void redStrip() {
    for (var i = 0; i < ledStripBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      ledStripBuffer.setRGB(i, 255, 0, 0);
   }
   

  }

  public void offStrip() {
    for (var i = 0; i < ledStripBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      ledStripBuffer.setRGB(i, 0, 0, 0);
   }
   
   ledStrip.setData(ledStripBuffer);


  }

  /**
   * Calculate and set the power to apply to the left and right
   * drive motors.
   * 
   * @param forward Desired forward speed. Positive is forward.
   * @param turn    Desired turning speed. Positive is counter clockwise from
   *                above. here?
   */

  public void setDriveMotors(double forward, double turn) {
    SmartDashboard.putNumber("drive forward power (%)", forward);
    SmartDashboard.putNumber("drive turn power (%)", turn);

    /*
     * positive turn = counter clockwise, so the left side goes backwards
     */
    double left = forward - turn;
    double right = forward + turn;

    SmartDashboard.putNumber("drive left power (%)", left);
    SmartDashboard.putNumber("drive right power (%)", right);

    // see note above in robotInit about commenting these out one by one to set
    // directions.
    driveLFTalon.set(ControlMode.PercentOutput, left);
    driveLRTalon.set(ControlMode.PercentOutput, left);
    driveRFTalon.set(ControlMode.PercentOutput, right);
    driveRRTalon.set(ControlMode.PercentOutput, right);
  }

  /**
   * Set the arm output power. Positive is out, negative is in.
   * 
   * @param percent
   */
  public void setArmMotor(double percent) {
    arm.set(percent);
    SmartDashboard.putNumber("arm power (%)", arm.getBusVoltage());
    SmartDashboard.putNumber("arm motor current (amps)", arm.getStatorCurrent());
    SmartDashboard.putNumber("arm motor temperature (C)", arm.getTemperature());
  }

  /**
   * Set the arm output power.
   * 
   * @param percent desired speed
   * @param amps    current limit
   */
  public void setIntakeMotor(double percent, int amps) {
    intake.set(percent);
    intake.setSmartCurrentLimit(amps);
    SmartDashboard.putNumber("intake power (%)", percent);
    SmartDashboard.putNumber("intake motor current (amps)", intake.getOutputCurrent());
    SmartDashboard.putNumber("intake motor temperature (C)", intake.getMotorTemperature());
  }

  /**
   * This method is called every 20 ms, no matter the mode. It runs after
   * the autonomous and teleop specific period methods.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Time (seconds)", Timer.getFPGATimestamp());
  }

  double autonomousStartTime;
  double autonomousIntakePower;

  @Override
  public void autonomousInit() {
    driveLFTalon.setNeutralMode(NeutralMode.Brake);
    driveLRTalon.setNeutralMode(NeutralMode.Brake);
    driveRFTalon.setNeutralMode(NeutralMode.Brake);
    driveRRTalon.setNeutralMode(NeutralMode.Brake);

    m_autoSelected = m_autoChooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);

    if (m_autoSelected == kConeAuto) {
      autonomousIntakePower = INTAKE_OUTPUT_POWER;
    } else if (m_autoSelected == kCubeAuto) {
      autonomousIntakePower = -INTAKE_OUTPUT_POWER;
    } else if (m_autoSelected == kConeBalanceAuto) {
      autonomousIntakePower = -INTAKE_OUTPUT_POWER;
    } else if (m_autoSelected == kCubeBalanceAuto) {
      autonomousIntakePower = -INTAKE_OUTPUT_POWER;
    }

    autonomousStartTime = Timer.getFPGATimestamp();
  }

  public void NoMobilityAuto() {
    setArmMotor(0.0);
    setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
    setDriveMotors(0.0, 0.0);
    return;
  }

  public void MobilityOnlyAuto() {

    double timeElapsed = Timer.getFPGATimestamp() - autonomousStartTime;

    if(timeElapsed < AUTO_DRIVE_TIME) {
      setArmMotor(0.0);
      setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
      setDriveMotors(-AUTO_DRIVE_SPEED, 0.0);
    } else {
      setArmMotor(0.0);
      setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
      setDriveMotors(0.5, 0.0);
    }

  }


  public void MobilityAuto() {

   double timeElapsed = Timer.getFPGATimestamp() - autonomousStartTime;

    if (timeElapsed < ARM_EXTEND_TIME_S) {
      setArmMotor(ARM_OUTPUT_POWER);
      setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
      setDriveMotors(0.0, 0.0);
    } else if (timeElapsed < ARM_EXTEND_TIME_S + AUTO_THROW_TIME_S) {
      setArmMotor(0.0);
      setIntakeMotor(autonomousIntakePower, INTAKE_CURRENT_LIMIT_A);
      setDriveMotors(0.0, 0.0);
    } else if (timeElapsed < ARM_EXTEND_TIME_S + AUTO_THROW_TIME_S + ARM_EXTEND_TIME_S) {
      setArmMotor(-ARM_OUTPUT_POWER);
      setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
      setDriveMotors(0.0, 0.0);
    } else if (timeElapsed < ARM_EXTEND_TIME_S + AUTO_THROW_TIME_S + ARM_EXTEND_TIME_S + AUTO_DRIVE_TIME) {
      setArmMotor(0.0);
      setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
      setDriveMotors(AUTO_DRIVE_SPEED, 0.0);
    } else {
      setArmMotor(0.0);
      setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
      setDriveMotors(0.0, 0.0);
    }

  }

  public void MobilityBalance() {

    double timeElapsed = Timer.getFPGATimestamp() - autonomousStartTime;

    if (timeElapsed < ARM_EXTEND_TIME_S) {
      //Arm extends
      setArmMotor(ARM_OUTPUT_POWER);
      setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
      setDriveMotors(0.0, 0.0);
    } else if (timeElapsed < ARM_EXTEND_TIME_S + AUTO_THROW_TIME_S) {
      //Drops game piece
      setArmMotor(0.0);
      setIntakeMotor(autonomousIntakePower, INTAKE_CURRENT_LIMIT_A);
      setDriveMotors(0.0, 0.0);
    } else if (timeElapsed < ARM_EXTEND_TIME_S + AUTO_THROW_TIME_S + ARM_EXTEND_TIME_S) {
      //Arm retracts
      setArmMotor(-ARM_OUTPUT_POWER);
      setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
      setDriveMotors(0.0, 0.0);
    } else if (timeElapsed < ARM_EXTEND_TIME_S + AUTO_THROW_TIME_S + ARM_EXTEND_TIME_S + AUTO_DRIVE_TIME) {
      //Robot drives in reverse
      setArmMotor(0.0);
      setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
      setDriveMotors(AUTO_DRIVE_SPEED, 0.0);
    } else if (timeElapsed < ARM_EXTEND_TIME_S + AUTO_THROW_TIME_S + ARM_EXTEND_TIME_S + AUTO_DRIVE_TIME) {
      //Robot drives forward 
      setArmMotor(0.0);
      setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
      setDriveMotors(AUTO_DRIVE_SPEED, 0.0);
    } else if (timeElapsed < ARM_EXTEND_TIME_S + AUTO_THROW_TIME_S + ARM_EXTEND_TIME_S + AUTO_DRIVE_TIME + AUTO_DRIVE_TIME) {
      //Balance on charging pad
      if (pitch < 0) {
        setDriveMotors(-AUTO_DRIVE_SPEED, 0.0);
      } else if (pitch > 0) {
        setDriveMotors(AUTO_DRIVE_SPEED, 0.0);
      } else if (pitch < 0) {
        setDriveMotors(0.0, 0.1);
      }
    } else {
      //Stops moving
      setArmMotor(0.0);
      setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
      setDriveMotors(0.0, 0.0);
    }
  }

  @Override
  public void autonomousPeriodic() {

    if (m_autoSelected == kNoMobilityAuto) {
      NoMobilityAuto();
    }

    if (m_autoSelected == kConeAuto) {
      MobilityAuto();
    }

    if (m_autoSelected == kCubeAuto) {
      MobilityAuto();
    }

    if (m_autoSelected == kConeBalanceAuto) {
      MobilityBalance();
    }

    if (m_autoSelected == kCubeBalanceAuto) {
      MobilityBalance();
    }

    if (m_autoSelected == kCubeBalanceAuto) {
      MobilityBalance();
    }

  }

  /**
   * Used to remember the last game piece picked up to apply some holding power.
   */
  static final int CONE = 1;
  static final int CUBE = 2;
  static final int NOTHING = 3;
  int lastGamePiece;

  @Override
  public void teleopInit() {
    driveLFTalon.setNeutralMode(NeutralMode.Coast);
    driveLRTalon.setNeutralMode(NeutralMode.Coast);
    driveRFTalon.setNeutralMode(NeutralMode.Coast);
    driveRRTalon.setNeutralMode(NeutralMode.Coast);

    lastGamePiece = NOTHING;
  }

  @Override
  public void teleopPeriodic() {
    double armPower;
    if (Gpad.getRawButton(7)) {
      // lower the arm
      armPower = -ARM_OUTPUT_POWER;
    } else if (Gpad.getRawButton(5)) {
      // raise the arm
      armPower = ARM_OUTPUT_POWER;
    } else {
      // do nothing and let it sit where it is
      armPower = 0.0;
    }
    setArmMotor(armPower);

    double intakePower;
    int intakeAmps;
    if (Gpad.getRawButton(8)) {
      // cube in or cone out
      intakePower = INTAKE_OUTPUT_POWER;
      intakeAmps = INTAKE_CURRENT_LIMIT_A;
      lastGamePiece = CUBE;
    } else if (Gpad.getRawButton(6)) {
      // cone in or cube out
      intakePower = -INTAKE_OUTPUT_POWER;
      intakeAmps = INTAKE_CURRENT_LIMIT_A;
      lastGamePiece = CONE;
    } else if (lastGamePiece == CUBE) {
      intakePower = INTAKE_HOLD_POWER;
      intakeAmps = INTAKE_HOLD_CURRENT_LIMIT_A;
    } else if (lastGamePiece == CONE) {
      intakePower = -INTAKE_HOLD_POWER;
      intakeAmps = INTAKE_HOLD_CURRENT_LIMIT_A;
    } else {
      intakePower = 0.0;
      intakeAmps = 0;
    }
    setIntakeMotor(intakePower, intakeAmps);

    if(Gpad.getRawButton(2)) {

      yellowStrip();

    } else if(Gpad.getRawButton(4)) {

      purpleStrip();

    } else {

      offStrip();

    }

    tankDrive.tankDrive(JoystickLeft.getRawAxis(1), JoystickRight.getRawAxis(1));

    /*
     * Negative signs here because the values from the analog sticks are backwards
     * from what we want. Forward returns a negative when we want it positive.
     */

  }

}