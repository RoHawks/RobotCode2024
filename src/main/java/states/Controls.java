package states;


public interface Controls 
{
    public boolean GetForceIntakingMode();
    public boolean GetForceEjectionMode();

    public boolean IsPieceInIntake();

    public boolean GetPrepareForHighGoalManual();
    public boolean GetPrepareForHighGoalDriveBy(); 
    public boolean GetPrepareForLowGoal();
    public boolean GetPrepareForAutoAim();

    public boolean GetStartShootingSequence();


    public boolean GetTrackScoringAngle();
    public double GetSwerveXComponent();
    public double GetSwerveYComponent();
    public double GetSwerveLinearSpeed();
    public double GetSwerveRotationalSpeed();
    public boolean GetSwerveBrakes();

    public boolean GetSwerveModeSwitch();

    public boolean GetManualControlsMode();

    public enum SwerveNudgingDirection
    {
        NONE,
        NORTH,
        SOUTH,
        EAST,
        WEST
    }

    public SwerveNudgingDirection GetSwerveNudgingDirection();

    public boolean GetManualAimCounterClockwise();
    public boolean GetManualAimClockwise();

    public boolean GetReleaseGamePiece();

    public void TurnOnVibrate();
    public void TurnOffVibrate();


}