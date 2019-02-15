/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
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
  WPI_TalonSRX driveOne = new WPI_TalonSRX(1);
  WPI_TalonSRX driveTwo = new WPI_TalonSRX(2);
  WPI_TalonSRX driveThree = new WPI_TalonSRX(3);
  WPI_TalonSRX driveFour = new WPI_TalonSRX(4);
  WPI_TalonSRX armOne = new WPI_TalonSRX(5);
  WPI_TalonSRX armTwo = new WPI_TalonSRX(6);
  WPI_VictorSPX intake = new WPI_VictorSPX(7);
  DoubleSolenoid hook = new DoubleSolenoid(7, 6);
  Solenoid lift = new Solenoid(5);
  SpeedControllerGroup m_right = new SpeedControllerGroup(driveOne, driveThree);
  SpeedControllerGroup m_left = new SpeedControllerGroup(driveTwo, driveFour);
  SpeedController arm = new SpeedControllerGroup(armOne, armTwo);
  DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);
  private Joystick m_stick = new Joystick(0);
  boolean upButt;
  boolean downButt;
  boolean shootButt;
  boolean hookButt;
  boolean liftButt;
  
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    driveOne.set(ControlMode.PercentOutput, 0);
    driveTwo.set(ControlMode.PercentOutput, 0);
    driveThree.set(ControlMode.PercentOutput, 0);
    driveFour.set(ControlMode.PercentOutput, 0);
    armOne.set(ControlMode.PercentOutput, 0);
    armTwo.set(ControlMode.PercentOutput, 0);
    armOne.setNeutralMode(NeutralMode.Brake);
    armTwo.setNeutralMode(NeutralMode.Brake);
    intake.set(ControlMode.PercentOutput, 0);

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
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    m_drive.arcadeDrive(m_stick.getY(), m_stick.getX());
    upButt = m_stick.getRawButton(5);
    downButt = m_stick.getRawButton(3);
    shootButt = m_stick.getRawButton(1);
    if(upButt){
      arm.set(0.5);
    } else {
      if(downButt){
        arm.set(-0.05);
      } else {
        arm.set(0);
      }
    }
    if(shootButt){
      intake.set(0.8);
    } else {
      intake.set(0);
    }
    hookButt = m_stick.getRawButton(4);
    if(hookButt){
      hook.set(DoubleSolenoid.Value.kForward);
    } else {
      hook.set(DoubleSolenoid.Value.kReverse);
    }
    liftButt = m_stick.getRawButton(9);
    lift.set(liftButt);
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
