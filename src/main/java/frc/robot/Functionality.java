package frc.robot;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import states.Controls;
import states.ShooterMode;

public class Functionality {
    public static void configureSparkMaxCustomizable(CANSparkMax pSparkMax, int pCurrentLimit, double pOpenLoopRampRate)
    {
        pSparkMax.restoreFactoryDefaults(); 
        pSparkMax.setSmartCurrentLimit(pCurrentLimit);    
        pSparkMax.setOpenLoopRampRate(pOpenLoopRampRate);
        pSparkMax.burnFlash();
    }


    public static void configureSparkMaxWeak(CANSparkMax pSparkMax)
    {
        pSparkMax.restoreFactoryDefaults(); 
        pSparkMax.setSmartCurrentLimit(20);    
        pSparkMax.setOpenLoopRampRate(0.2);
        pSparkMax.burnFlash();
    }
    
    public static void configureSparkMaxStrong(CANSparkMax pSparkMax)
    {
        pSparkMax.restoreFactoryDefaults(); 
        pSparkMax.setSmartCurrentLimit(40);    
        pSparkMax.setOpenLoopRampRate(0.2);
        pSparkMax.burnFlash();
    }


    public static ShooterMode checkForShootingPreperationButtons(Controls pControls, ShooterMode pPreviouShooterMode)
    {
        ShooterMode returnValue = pPreviouShooterMode;
        if(pControls.GetPrepareForHighGoalManual())
        {
            returnValue = ShooterMode.HighGoalManual;
            LimelightManager.GetInstance().SetPipeline(LimelightManager.PIPELINE_FOR_DRIVE_BY_SHOOTING);
        }
        else if(pControls.GetPrepareForHighGoalDriveBy())
        {
            returnValue = ShooterMode.HighGoalDriveBy;
            LimelightManager.GetInstance().SetPipeline(LimelightManager.PIPELINE_FOR_DRIVE_BY_SHOOTING);
        }
        else if(pControls.GetPrepareForLowGoal())
        {
            returnValue = ShooterMode.LowGoal;
            LimelightManager.GetInstance().SetPipeline(LimelightManager.PIPELINE_FOR_DRIVE_BY_SHOOTING);
        }
        else if(pControls.GetPrepareForAutoAim())
        {
            returnValue = ShooterMode.AutoAim;
            LimelightManager.GetInstance().SetPipeline(LimelightManager.PIPELINE_FOR_DRIVE_BY_SHOOTING);
        }
        return returnValue;
    }

    

    

}
