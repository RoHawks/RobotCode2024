package states;

public class ScoringInstructions 
{
    private GamePieceLocation mGamePieceLocation;
    private GamePieces mGamePiece;
    private boolean mManualAimOverride;
    private double mManualAimDirection;

    public ScoringInstructions(GamePieceLocation pGamePieceLocation, GamePieces pGamePiece, double pManualAimDirection, boolean pManualAimOverride)
    {
        mGamePieceLocation = pGamePieceLocation;
        mGamePiece = pGamePiece;
        mManualAimOverride = pManualAimOverride;
        mManualAimDirection = pManualAimDirection;
    }

    public GamePieceLocation GetGamePieceLocation()
    {
        return mGamePieceLocation;
    }

    public GamePieces GetGamePiece()
    {
        return mGamePiece;
    }

    public boolean GetManualAimOverride()
    {
        return mManualAimOverride;
    }

    public double GetManualAimDirection()
    {
        return mManualAimDirection;
    }
    
}
