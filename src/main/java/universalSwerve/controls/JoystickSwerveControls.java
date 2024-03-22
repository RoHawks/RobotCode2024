package universalSwerve.controls;

import edu.wpi.first.wpilibj.XboxController;
import universalSwerve.utilities.SwerveNudgingDirection;

public class JoystickSwerveControls implements ISwerveControls
{
    private XboxController mMainController;

    public JoystickSwerveControls( XboxController pMainController)
    {
        mMainController = pMainController;
    }

    public double GetSwerveXComponent()
    {
        return mMainController.getLeftX();
    }

    public double GetSwerveYComponent()
    {
        return mMainController.getLeftY();
    }

    public double GetSwerveLinearSpeed()
    {
        //square to allow for easy speed control on low speeds
        double triggerValue = mMainController.getRightTriggerAxis();
        return (triggerValue * triggerValue) * 0.93; //ATS
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

    public boolean GetSwerveBrakes() {
        return false; //GIVE ACTUAL LATER
    }

    public boolean GetTrackSpecificAngle()
    {
        return false;
    }
    
    public double GetSpecificAngleToTrack()
    {
        return 0;
    }

}