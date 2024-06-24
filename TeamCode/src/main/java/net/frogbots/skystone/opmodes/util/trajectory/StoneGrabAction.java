//package net.frogbots.skystone.opmodes.util.trajectory;
//
//import net.frogbots.skystone.opmodes.auto.Globals;
//
//public class StoneGrabAction implements MovementPerformer
//{
//    AutoClaw autoClaw;
//
//    @Override
//    public void run()
//    {
//        autoClaw = Globals.autoClaw;
//
//
//        autoClaw.pinch.setPosition(AutoClaw.PINCH_CLAMP);
//
//        try
//        {
//            Thread.sleep(500);
//        } catch (InterruptedException e)
//        {
//            Thread.currentThread().interrupt();
//        }
//
//        autoClaw.pivot.setPosition(AutoClaw.PIVOT_UP);
//
//        try
//        {
//            Thread.sleep(500);
//        } catch (InterruptedException e)
//        {
//            Thread.currentThread().interrupt();
//        }
//    }
//
//    public StoneGrabAction setPivot(boolean up)
//    {
//        return this;
//    }
//}
