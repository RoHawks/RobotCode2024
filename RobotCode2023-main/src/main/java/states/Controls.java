package states;


public interface Controls 
{
    public boolean GetIsLowGoalMode();
    public boolean GetSwitchAutoOption();
    public boolean GetSwitchAutoDirection();
    public GamePieces GetGamePieceMode();

    public boolean GetForceIntakingMode();
    public boolean GetForceHoldingMode();
    public boolean GetForceEjectionMode();


    public boolean GetPrepareForHigh();
    public boolean GetPrepareForMiddle();    
    public boolean GetPrepareForLow();

    public boolean GetInvokeScoringRoutine();
    
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