package robotcode.autonomous;

public abstract class AAction
{

    private long mEntryTimestamp;

    public void EnterAction()
    {
         mEntryTimestamp = System.currentTimeMillis();
    }

    public boolean HasTimeElapsed(long pMilliseconds)
    {
        return System.currentTimeMillis() > mEntryTimestamp + pMilliseconds;
    }
    //returns true iff the action is complete
    public abstract boolean Run();

    public abstract void EndAction();


}