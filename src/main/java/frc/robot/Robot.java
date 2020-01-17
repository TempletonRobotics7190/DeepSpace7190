/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends IterativeRobot {
  //Create motor controller objects
  WPI_TalonSRX driveOne = new WPI_TalonSRX(1);
  WPI_TalonSRX driveTwo = new WPI_TalonSRX(2);
  WPI_TalonSRX driveThree = new WPI_TalonSRX(3);
  WPI_TalonSRX driveFour = new WPI_TalonSRX(4);
  WPI_VictorSPX armOne = new WPI_VictorSPX(7);
  WPI_TalonSRX armTwo = new WPI_TalonSRX(6);
  WPI_TalonSRX intake = new WPI_TalonSRX(5);
  //Create solenoid objects
  DoubleSolenoid hook = new DoubleSolenoid(7, 6);
  Solenoid backLift = new Solenoid(5);
  Solenoid frontLift = new Solenoid(0);
  //Group speed controllers
  SpeedControllerGroup m_right = new SpeedControllerGroup(driveOne, driveThree);
  SpeedControllerGroup m_left = new SpeedControllerGroup(driveTwo, driveFour);
  //Create drive object
  DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);
  //Create joystick object
  private Joystick m_stick = new Joystick(0);
  private Joystick weeb = new Joystick(1);
  
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    //Set Talon motor controllers to 0% output
    driveOne.set(ControlMode.PercentOutput, 0);
    driveTwo.set(ControlMode.PercentOutput, 0);
    driveThree.set(ControlMode.PercentOutput, 0);
    driveFour.set(ControlMode.PercentOutput, 0);
    armOne.set(ControlMode.PercentOutput, 0);
    armTwo.set(ControlMode.PercentOutput, 0);
    armOne.setNeutralMode(NeutralMode.Brake);
    armTwo.setNeutralMode(NeutralMode.Brake);
    armOne.follow(armTwo);
    intake.set(ControlMode.PercentOutput, 0);
    intake.setNeutralMode(NeutralMode.Brake);
    //Set intake current limit to 35 amps and stay off for 1 ms
    intake.configPeakCurrentLimit(15, 1);
    intake.enableCurrentLimit(true);

    /* Config the sensor used for Primary PID and sensor direction */
    armTwo.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 
    Constants.kPIDLoopIdx,
Constants.kTimeoutMs);

/* Ensure sensor is positive when output is positive */
armTwo.setSensorPhase(Constants.kSensorPhase);

/**
* Set based on what direction you want forward/positive to be.
* This does not affect sensor phase. 
*/ 
armTwo.setInverted(Constants.kMotorInvert);

/* Config the peak and nominal outputs, 12V means full */
armTwo.configNominalOutputForward(0, Constants.kTimeoutMs);
armTwo.configNominalOutputReverse(0, Constants.kTimeoutMs);
armTwo.configPeakOutputForward(0.3, Constants.kTimeoutMs);
armTwo.configPeakOutputReverse(-0.3, Constants.kTimeoutMs);

/**
* Config the allowable closed-loop error, Closed-Loop output will be
* neutral within this range. See Table in Section 17.2.1 for native
* units per rotation.
*/
armTwo.configAllowableClosedloopError(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

/* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
armTwo.config_kF(Constants.kPIDLoopIdx, 0, Constants.kTimeoutMs);
armTwo.config_kP(Constants.kPIDLoopIdx, 0.3, Constants.kTimeoutMs);
armTwo.config_kI(Constants.kPIDLoopIdx, 0, Constants.kTimeoutMs);
armTwo.config_kD(Constants.kPIDLoopIdx, 0, Constants.kTimeoutMs);

/**
* Grab the 360 degree position of the MagEncoder's absolute
* position, and intitally set the relative sensor to match.
*/
int absolutePosition = armTwo.getSensorCollection().getPulseWidthPosition();

/* Mask out overflows, keep bottom 12 bits */
absolutePosition &= 0xFFF;
if (Constants.kSensorPhase) { absolutePosition *= -1; }
if (Constants.kMotorInvert) { absolutePosition *= -1; }

/* Set the quadrature (relative) sensor to match absolute */
armTwo.setSelectedSensorPosition(absolutePosition, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

    new Thread(() -> {
      UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
      camera.setResolution(256, 144);

      CvSink cvSink = CameraServer.getInstance().getVideo();
      CvSource outputStream = CameraServer.getInstance().putVideo("Blur", 256, 144);

      Mat source = new Mat();
      Mat output = new Mat();

      while(!Thread.interrupted()) {
          cvSink.grabFrame(source);
          Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
          outputStream.putFrame(output);
      }
  }).start();
    
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    //Fetch joystick and button values
    //HALF SPEEN MYBE
    double yStick = m_stick.getY() * 0.5;
    double xStick = m_stick.getX() * 0.5;
    m_drive.arcadeDrive(yStick, xStick);
    boolean upButt = weeb.getRawButton(3);
    boolean downButt = weeb.getRawButton(4);
    boolean fastDown = weeb.getRawButton(6);
    boolean rocket = weeb.getRawButton(1);
    boolean shootButt = m_stick.getRawButton(1);
    boolean intakeButt = m_stick.getRawButton(2);
    //Arm commands
    if(upButt){
      armTwo.set(ControlMode.Position, -1000);
    } else {
      if(downButt){
        armTwo.set(ControlMode.Position, -1);
      } else { if(fastDown) {
        armTwo.set(ControlMode.PercentOutput, -0.1);
      } else { if(rocket) {
        armTwo.set(ControlMode.Position, -2000);
      } else {
        armTwo.set(ControlMode.PercentOutput, 0);
          }
        }
      }
    }
    //Intake commands
    if(shootButt){
      intake.set(0.4);
    } else {
      if(intakeButt){
        intake.set(-0.4);
      } else {
        intake.set(0);
      }
    }
    //Hook commands
    boolean hookButt = m_stick.getRawButton(3);
    if(!hookButt){
      hook.set(DoubleSolenoid.Value.kForward);
    } else {
      hook.set(DoubleSolenoid.Value.kReverse);
    }
    //Fetch climb values
    boolean backLiftButt = m_stick.getRawButton(9);
    boolean frontLiftButt = m_stick.getRawButton(7);
    //Set lift solenoids
    backLift.set(backLiftButt);
    frontLift.set(frontLiftButt);
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    //Fetch joystick and button values
    m_drive.arcadeDrive(m_stick.getY(), m_stick.getX());
    boolean upButt = weeb.getRawButton(3);
    boolean downButt = weeb.getRawButton(4);
    boolean fastDown = weeb.getRawButton(5);
    boolean fastUp = weeb.getRawButton(6);
    boolean rocket = weeb.getRawButton(1);
    boolean shootButt = m_stick.getRawButton(1);
    boolean intakeButt = m_stick.getRawButton(2);
    //Arm commands
    if(upButt){
      armTwo.set(ControlMode.Position, -800);
    } else {
      if(downButt){
        armTwo.set(ControlMode.Position, -1);
      } else { if(fastDown) {
        armTwo.set(ControlMode.PercentOutput, -0.1);
      } else { if(rocket) {
        armTwo.set(ControlMode.Position, -2000);
      } else { if(fastUp) {
        armTwo.set(ControlMode.PercentOutput, 0.2);
      } else {
        armTwo.set(ControlMode.PercentOutput, 0);
      }
          }
        }
      }
    }
    //Intake commands
    if(shootButt){
      intake.set(0.4);
    } else {
      if(intakeButt){
        intake.set(-0.4);
      } else {
        intake.set(0);
      }
    }
    //Hook commands
    boolean hookButt = m_stick.getRawButton(3);
    if(!hookButt){
      hook.set(DoubleSolenoid.Value.kForward);
    } else {
      hook.set(DoubleSolenoid.Value.kReverse);
    }
    //Fetch climb values
    boolean backLiftButt = m_stick.getRawButton(9);
    boolean frontLiftButt = m_stick.getRawButton(7);
    //Set lift solenoids
    backLift.set(backLiftButt);
    frontLift.set(frontLiftButt);
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
