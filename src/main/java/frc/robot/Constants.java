package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Constants {

    // Intake Constants
    public static final double NORMAL_INTAKING_SPEED_CONVEYORS = 1;
    public static final double NORMAL_INTAKING_SPEED_INTAKE_ROLLER = 1;
    public static final double EJECTING_SPEED = -0.8;
    public static final double INTAKE_ROLLERS_GENTLE_ROLLING_BACKWARDS_SPEED = -0.2;
    public static final double INTAKE_CONVEYORS_BACKUP_SPEED = -0.1;
    
    public static final double ROTATIONS_FOR_CONVEYORS_TO_BACK_UP_TO = -0.5;

    // Shooter Constants
    public static final double ANGLE_ADJUSTMENT = -2.7;
    public static final double INTAKING_ANGLE = 20 + ANGLE_ADJUSTMENT;
    public static final double LOW_GOAL_ANGLE = 55 + ANGLE_ADJUSTMENT;
    public static final double HIGH_GOAL_ANGLE = 64 + ANGLE_ADJUSTMENT;

    public static final double DEFAULT_AUTO_AIM_ANGLE = 46.3419;

    public static final double INITIAL_SHOOTER_ANGLE_TO_ACCOUNT_FOR = 16.2;
    
    public static final double Z_VELOCITY_COMPENSATION = -17;


    public static final double SHOOTER_HIGH_TOP_DEFAULT_SPEED = 46.0; // at Townsend this was 46.0 but it doesn't seem like we can realistically hold 46
    public static final double SHOOTER_HIGH_BOTTOM_DEFAULT_SPEED = 46.0 * 3.0/4.0; // at Townsend this was 46.0 * 3.0/4.0 but it doesn't seem like we can realistically hold 46

    public static final double SHOOTER_LOW_TOP_DEFAULT_SPEED = 46.0 * 0.5; // 46.0
    public static final double SHOOTER_LOW_BOTTOM_DEFAULT_SPEED = 46.0 * 3.0/4.0 * 0.5; //46.0 * 3.0/4.0

    public static final double ANGLER_ROTATIONS_TO_ANGLES = 360.0 / 810.0; //360 degrees / ((27:1 gearbox) *(31:1 gearbox) (full revolution)) 
    public static final double DRIVE_BY_SHOOTNG_DISTANCE_ERROR_MARGIN = 0.15; // arbitrary
    public static final double DRIVE_BY_SHOOTNG_DISTANCE_ERROR_MARGIN_WHEN_STOPPED = 0.04; // arbitrary

    
    // public static final double 13.75/17.6



    public static double LOW_GOAL_ROTATION = 270;
    public static final double HIGH_GOAL_ROTATION = 180;  // remember to set this back to 0 for real game.  180 is for testing where you can ssee the foal.
    public static final double ANGLER_ACCEPTABLE_ERROR = 15.0;


    

    public static void setLowGoalRotation()
    {
        try 
        {
            String currentAlliance = DriverStation.getAlliance().get().toString();
            if (currentAlliance.equals("Red"))
            {
                LOW_GOAL_ROTATION = 270;
            }
            else
            {
                LOW_GOAL_ROTATION = 90;
            }
        }
        catch (Exception e)
        {
            SmartDashboard.putString("ERROR IN SETTING LOW GOAL ROTATION", "TRUE");
            LOW_GOAL_ROTATION = 90;
        }
        
       
    }
}



