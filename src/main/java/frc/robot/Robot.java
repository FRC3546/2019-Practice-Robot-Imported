// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// This imports a SPI bus port
import edu.wpi.first.wpilibj.SPI;

// Formerly known as iterative, this is the type of drive for the robot
import edu.wpi.first.wpilibj.TimedRobot;

// Imports tankdrive libraries
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

// Timer import
import edu.wpi.first.wpilibj.Timer;

// THIS IS THE CONTROLLER IMPORT (It is important and took us a while to find)
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;

// Joystick support for the joysticks (duh)
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

// Nav x Gyro import
import com.kauailabs.navx.frc.AHRS;

// Camera server import
import edu.wpi.first.cameraserver.CameraServer;

// Double Solenoid import
import edu.wpi.first.wpilibj.DoubleSolenoid;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

// Pneumatics import
import edu.wpi.first.wpilibj.PneumaticsModuleType;

// Smartdashboard imports
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Shuffleboard import
import edu.wpi.first.wpilibj.shuffleboard.*;

 /* This is a demo program showing the use of the RobotDrive class, specifically it contains the code
  necessary to operate a robot with tank drive. */

public class Robot extends TimedRobot {
  private DifferentialDrive m_myRobot;
  private Joystick m_leftStick;
  private Joystick m_rightStick;
  /*private JoystickButton Lbutton1;
  private JoystickButton Lbutton2;
  private JoystickButton Lbutton3;
  private JoystickButton Lbutton4;
  private JoystickButton Lbutton5;
  private JoystickButton Lbutton6;
  private JoystickButton Lbutton7;
  private JoystickButton Lbutton8;*/
  
  // Variables for Solenoid toggling
  boolean toggleOnSolenoid = false;
  boolean toggleHeldSolenoid = false;

  boolean isInverted = false; //From starting posistion

  // Here we define m_left and m_left for our motor controllers
  private VictorSP m_left = new VictorSP(6);
  private VictorSP m_right = new VictorSP(2);

  // Variable for the timer class
  private final Timer m_timer = new Timer();

  private DoubleSolenoid Arm;
  
  // This code sets up stuff for the autonomous selector
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private static final String kGyro = "Gyro";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  // This code creates a new gyro object of the AHRS class
  AHRS gyro = new AHRS(SPI.Port.kMXP);

  final double kP = 1;


  @Override
  public void robotInit() {
    
    // DifferentialDrive is another word for tank drive and is used on the 2019 robot
    m_myRobot = new DifferentialDrive(m_left, m_right);
    
    // Here we pair the motor controllers with the joysticks
    m_leftStick = new Joystick(1);
    m_rightStick = new Joystick(0);

    // These are the different autonomous programs that can be selected in the Shuffleboard or Smart dashboard
    m_chooser.setDefaultOption("kAutoNameDefault", kDefaultAuto);
    m_chooser.addOption("kAutoNameCustom", kCustomAuto);
    m_chooser.addOption("Gyro", kGyro);
    
    // Adds the autonomous programs into one of the dashboards
    SmartDashboard.putData("Auto choices", m_chooser);
    
    // Adds gyro to the Shuffleboard
    Shuffleboard.getTab("Example tab").add(gyro);


     // These are the left joystick buttons
      // Lbutton1 = new JoystickButton(m_leftStick, 0);
      // Lbutton2 = new JoystickButton(m_leftStick, 2);
      // Lbutton3 = new JoystickButton(m_leftStick, 3);
      // Lbutton4 = new JoystickButton(m_leftStick, 4);
      // Lbutton5 = new JoystickButton(m_leftStick, 5);
      // Lbutton6 = new JoystickButton(m_leftStick, 6);
      // Lbutton7 = new JoystickButton(m_leftStick, 7);
      // Lbutton8 = new JoystickButton(m_leftStick, 8);
    
    
    // Inverts the controls for Teleop  
    m_left.setInverted(true);
    m_right.setInverted(true);

    // Double Solenoid being defined as "Arm"
    Arm = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);

    // Initialize Camera Server
    CameraServer.startAutomaticCapture();
  }


/* This function is run once each time the robot enters autonomous mode. This function is also imporant
because it resets the timer to 0 and then starts the timer at the begining of the autonomous period
so our robot knows to only drive for 2 seconds */
@Override
public void autonomousInit() {
    

  m_autoSelected = m_chooser.getSelected();

  System.out.println("Auto selected: " + m_autoSelected);

  // Resets our timer to 0 then starts the timer
  m_timer.reset();
  m_timer.start();
}

  /** This function is called periodically during autonomous. */
@Override
public void autonomousPeriodic() {

  // This is our custom code to drive forward, do a 180 and finally, drive back
  switch (m_autoSelected) {
    case kCustomAuto:
      // Drive forward for 2 seconds
      if (m_timer.get() < 2)
      {
        m_myRobot.tankDrive(-0.5, -0.5);
      }
      
      // Should do a 180 degree turn but depends on multiple variables (Basically this code is old)
      else if (m_timer.get() < 3.25)
      {
        m_myRobot.tankDrive(0.7, -0.7);
      }
      
      // Drives back to the starting position (if the turn works)
      else if(m_timer.get() < 5)
      {
        m_myRobot.tankDrive(-0.5, -0.5);
      }
      else {
        m_myRobot.stopMotor(); // stops the robot
      }  
      break;
    
    // This is the default autonomous
    case kDefaultAuto:
    default:
     
      // This is currently intended to do nothing
    
      break;
    
    // This is the gyro autonomous 
    case kGyro:
      
      // Sets the angle we want to move to
      double error = 45 - gyro.getAngle();
      
      // Turns to the targeted angle (the kp value may need to be adjusted depending on the targeted
      // gyro angle
      m_myRobot.tankDrive(-(kP * error), (kP * error));
      

      break;
    }
  }

// This inverts the teleop controls with the press of a button
public void updateInversionValue()
  {
    if(m_leftStick.getRawButton(3))
    {
      isInverted = true;
    }
    if(m_leftStick.getRawButton(4))
    {
      isInverted = false;
    }
  }


  // Toggles the values of the solenoids when the joystick trigger is held in
  public void updateToggle()
  {
      if(m_leftStick.getRawButton(1)){
          if(!toggleHeldSolenoid){
              toggleOnSolenoid = !toggleOnSolenoid;
              toggleHeldSolenoid = true;
          }
      }else{
          toggleHeldSolenoid = false;
          toggleOnSolenoid = false;}
      }



  //This is the teleoperated mode that you select from the Driver Station
  @Override
  public void teleopPeriodic() {
    
    // Updates the inversion (checks for whether the robot controls are inverted or not)
    updateInversionValue();


    if(m_rightStick.getRawButton(2))
    {
      double magnitude = (m_leftStick.getY() + m_rightStick.getY()) / 2;
      double leftStickValue = magnitude;
      double rightStickValue = magnitude;
      m_myRobot.tankDrive(leftStickValue,  rightStickValue);
    }
    else
    {
      
      // 
      if (!isInverted)
      {
        m_myRobot.tankDrive(m_leftStick.getY(), m_rightStick.getY());
      }
      else
      {
        m_myRobot.tankDrive((-1) * m_rightStick.getY(), (-1) * m_leftStick.getY());
      }
    }

    // m_myRobot.tankDrive(m_leftStick.getY(), m_rightStick.getY());
    

    // Boolean ArmButtonPressed = m_leftStick.getRawButton(3);
    // if (ArmButtonPressed) {
    //   toggleHeldSolenoid = true;
    // }


    // Important for the toggle you will see in a bit
    updateToggle();

    if (toggleOnSolenoid){
      Arm.set(kForward);
    }else{
      Arm.set(kReverse);
    }

    // Resets the gyro with the press of a button
    if (m_rightStick.getRawButton(3))
    {
      gyro.zeroYaw();
    }
    }


/*  
    
  if(toggleOn){
    Arm.set(kForward);
  }else{
    Arm.set(kReverse);}
    }
          
    
  
  public void updateToggle2()
  {
      if(m_leftStick.getRawButton(2)){
        if(!toggleHeld){
            toggleOn = !toggleOn;
            toggleHeld = true;
        }
      }else{
          toggleHeld = false;
      
*/
    

    /*Boolean ArmButtonReleased = m_leftStick.getRawButton(2);
    if (ArmButtonReleased) {
      Arm.set(kReverse);
    }*/

    //Lbutton1.whenPressed(Arm.toggle());
    
    //Lbutton2.whenPressed(() -> Arm.set(kReverse));

  }

  

