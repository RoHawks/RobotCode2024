package states;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import universalSwerve.utilities.SwerveNudgingDirection;


public class ApproachingClimberControls extends JoystickControlsWithSwerve {


    double mTimeEnteredClimbPrep; 

    public static ApproachingClimberControls Instance;

    public static void setInstanceInformation(XboxController pMainController, XboxController pAlternateController)
    {
        Instance = new ApproachingClimberControls(pMainController, pAlternateController);
        Instance.setTimeSinceEnteredClimbingState(System.currentTimeMillis());
    }

    private ApproachingClimberControls(XboxController pMainController, XboxController pAlternateController)
    {
        super(pMainController, pAlternateController);
        
    }

    public void setTimeSinceEnteredClimbingState(double pTime)
    {
        mTimeEnteredClimbPrep = pTime;
    }

    public double turnTimeIntoScalingValue(double pTime)
    {
        double scalingValue = Math.min(Math.max(0.5, 1 - (System.currentTimeMillis() - mTimeEnteredClimbPrep)/1000.0 * 0.5),1);
        SmartDashboard.putNumber("scalingVal", scalingValue);
        return scalingValue;
    }




    public double GetSwerveXComponent()
    {
        return mMainController.getLeftX() * turnTimeIntoScalingValue(mTimeEnteredClimbPrep);
    }

    public double GetSwerveYComponent()
    {
        return mMainController.getLeftY() * turnTimeIntoScalingValue(mTimeEnteredClimbPrep);
    }

    public double GetSwerveLinearSpeed()
    {
        //square to allow for easy speed control on low speeds
        double triggerValue = mMainController.getRightTriggerAxis();
        return 1.0 * triggerValue * triggerValue * turnTimeIntoScalingValue(mTimeEnteredClimbPrep);
    }

    public double GetSwerveRotationalSpeed()
    {
        //Let's try squaring this for finer grained slowcontrol
        double rawVal = mMainController.getRightX();
        return rawVal > 0 ? rawVal * rawVal : -1.0 * rawVal * rawVal;
    }


    public boolean GetSwerveModeSwitch()
    {
        return mMainController.getStartButton();
    }

    public SwerveNudgingDirection GetSwerveNudgingDirection()
    {

        if(mMainController.getAButton())
        {
            return SwerveNudgingDirection.SOUTH;
        }
        else if(mMainController.getXButton())
        {
            return SwerveNudgingDirection.EAST;
        }
        else if(mMainController.getYButton())
        {
            return SwerveNudgingDirection.NORTH;
        }
        else if(mMainController.getBButton())
        {
            return SwerveNudgingDirection.WEST;
        }
        else
        {
            return SwerveNudgingDirection.NONE;
        }
    }


    @Override
    public boolean GetSwerveBrakes() {
        return false; //GIVE ACTUAL LATER
    }

    public boolean GetManualControlsMode()
    {
        //SmartDashboard.putNumber("InJBC_LTR", mAlternateController.getLeftTriggerAxis());        
        return mAlternateController.getLeftTriggerAxis() > 0.5;
    }

    public boolean GetManualAimCounterClockwise()
    {
        return mAlternateController.getRightBumper();
    }
    public boolean GetManualAimClockwise()
    {
        return mAlternateController.getStartButton();
        
    }

    public boolean GetReleaseGamePiece()
    {
        return mMainController.getRightBumper();
    }

    public boolean GetIsLowGoalMode()
    {
        return mAlternateController.getLeftY() < -0.5;
    }

    public void TurnOnVibrate()
    {
        mMainController.setRumble(RumbleType.kBothRumble, 1);
    }
    public void TurnOffVibrate()
    {
        mMainController.setRumble(RumbleType.kBothRumble, 0);
    }

    @Override
    public boolean GetTrackSpecificAngle() {
       return false;
    }

    @Override
    public double GetSpecificAngleToTrack() {
        return -1;
    }


    
}
