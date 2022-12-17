//package net.frogbots.skystone.opmodes.util.trajectory;
//
//import net.frogbots.skystone.opmodes.auto.Globals;
//
//public class StoneReleaseAction implements MovementPerformer
//{
//    AutoClaw autoClaw;
//
//    @Override
//    public void run()
//    {
//        autoClaw = Globals.autoClaw;
//
//        autoClaw.pivot.setPosition(AutoClaw.PIVOT_RELEASE);
//
//        autoClaw.pinch.setPosition(AutoClaw.PINCH_OPEN);
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
//    }
//}
