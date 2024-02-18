// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Arrays;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DoubleLogEntry;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */

  public double[] getCameraPoseTargetSpace(String json)
  {
    // try {
      int idx1 = json.indexOf("t6c_ts");
      int idx2 = json.indexOf("t6r_fs");
      if (idx1 > 0 && idx2 > 0)
      {
        // SmartDashboard.putString("test", json.substring(idx1,idx2));
        String s1 = json.substring(idx1,idx2);
        s1 = s1.substring(9,s1.length()-3);
        String[] comma_split_list = s1.split(",");
        double[] double_array = new double[6];
        for (int i = 0; i < comma_split_list.length; i++)
        {
            // System.out.println(comma_split_list[i]);
            double_array[i] = Double.parseDouble(comma_split_list[i]);
        }
        // SmartDashboard.putString("should work", Arrays.toString(double_array));
        return double_array;
      }
      return null;
    // }
    // catch (Exception e) {
    //   throw e;
     
    // }
    
  }
  @Override
  public void teleopPeriodic() {

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry jsonNTE = table.getEntry("json");

    // String[] entries = {"botpose", "botpose_wpiblue", 
    //                     "botpose_wpired", "camerapose_targetspace", 
    //                     "targetpose_cameraspace", "targetpose_robotspace",
    //                     "botpose_targetspace", "camerapose_robotspace",
    //                     "tid"};

    // String currentEntry = entries[(int)((System.currentTimeMillis() / 3000) % entries.length)];

    // NetworkTableEntry tbp = table.getEntry(currentEntry);





    String json_dump = jsonNTE.getString("null");

    double[] camerapose_targetspace = getCameraPoseTargetSpace(json_dump);
    if (camerapose_targetspace == null)
    {
      camerapose_targetspace = new double[0];
    } 
    // double[] botPose = tbp.getDoubleArray(new double[6]);
    
    SmartDashboard.putString("json", json_dump);
    // SmartDashboard.putString("currentEntry", Arrays.toString(botPose));
    // SmartDashboard.putNumber("Limelights1", s1);

    // SmartDashboard.putNumber("LimelightBP1", botPose[0]);

    SmartDashboard.putString("CP_TS", Arrays.toString(camerapose_targetspace));
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
