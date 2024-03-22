package frc.robot;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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


    private static int mSelectionChangeCounter = 0;
    private static boolean mLastTimeWasSelecting = false;
    public static ShooterMode checkForShootingPreperationButtons(Controls pControls, ShooterMode pPreviouShooterMode)
    {
        ShooterMode returnValue = pPreviouShooterMode;
        boolean isSelecting = false;
        if(pControls.GetPrepareForHighGoalManual())
        {
            SmartDashboard.putString("SoundFileName", "SubwooferShot.wav");
            returnValue = ShooterMode.HighGoalManual;
            LimelightManager.GetInstance().SetPipeline(LimelightManager.PIPELINE_FOR_DRIVE_BY_SHOOTING);
            isSelecting = true;
        }
        else if(pControls.GetPrepareForHighGoalDriveBy())
        {
            SmartDashboard.putString("SoundFileName", "DriveByShooting.wav");
            returnValue = ShooterMode.HighGoalDriveBy;
            LimelightManager.GetInstance().SetPipeline(LimelightManager.PIPELINE_FOR_DRIVE_BY_SHOOTING);
            isSelecting = true;
        }
        else if(pControls.GetPrepareForLowGoal())
        {
            SmartDashboard.putString("SoundFileName", "LowGoalMode.wav");
            returnValue = ShooterMode.LowGoal;
            LimelightManager.GetInstance().SetPipeline(LimelightManager.PIPELINE_FOR_DRIVE_BY_SHOOTING);
            isSelecting = true;
        }
        else if(pControls.GetPrepareForAutoAim())
        {
            SmartDashboard.putString("SoundFileName", "AutoAimMode.wav");
            returnValue = ShooterMode.AutoAim;
            LimelightManager.GetInstance().SetPipeline(LimelightManager.PIPELINE_FOR_DRIVE_BY_SHOOTING);
            isSelecting = true;
        }

        if(isSelecting && !mLastTimeWasSelecting)
        {
            mSelectionChangeCounter++;
        }
        mLastTimeWasSelecting = isSelecting;
        SmartDashboard.putNumber("ModeSelectionCounter", mSelectionChangeCounter);
        return returnValue;
    }

    

    

}
