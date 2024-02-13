package universalSwerve.controls;

import universalSwerve.utilities.SwerveNudgingDirection;

public interface ISwerveControls
{

    public double GetSwerveXComponent();
    public double GetSwerveYComponent();
    public double GetSwerveLinearSpeed();
    public double GetSwerveRotationalSpeed();
    public boolean GetSwerveBrakes();
    public boolean GetSwerveModeSwitch();
    public boolean GetTrackSpecificAngle();
    public double GetSpecificAngleToTrack();


    public SwerveNudgingDirection GetSwerveNudgingDirection();


}