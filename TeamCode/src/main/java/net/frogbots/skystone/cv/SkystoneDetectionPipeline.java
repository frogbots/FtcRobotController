package net.frogbots.skystone.cv;

import net.frogbots.skystone.meta.misc.Alliance;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class SkystoneDetectionPipeline extends OpenCvPipeline
{
    Mat submat1, submat2, submat3;
    Mat YCrCb = new Mat();
    Mat Cb = new Mat();
    Point sub1pointA;
    Point sub1pointB;

    Point sub2pointA;
    Point sub2pointB;

    Point sub3pointA;
    Point sub3pointB;

    int avg1, avg2, avg3;

    private volatile SkystonePosition position = SkystonePosition.RIGHT;

    public SkystoneDetectionPipeline()
    {
//        if(Globals.alliance == Alliance.RED)
//        {
//            sub1pointA = new Point(98,109);
//            sub1pointB = new Point(sub1pointA.x + 5, sub1pointA.y + 5);
//
//            sub2pointA = new Point(98,181);
//            sub2pointB = new Point(sub2pointA.x + 5, sub2pointA.y + 5);
//
//            sub3pointA = new Point(98,253);
//            sub3pointB = new Point(sub3pointA.x + 5, sub3pointA.y + 5);
//        }
//        else if(Globals.alliance == Alliance.BLUE)
//        {
//            sub1pointA = new Point(98,37);
//            sub1pointB = new Point(sub1pointA.x + 5, sub1pointA.y + 5);
//
//            sub2pointA = new Point(98,98);
//            sub2pointB = new Point(sub2pointA.x + 5, sub2pointA.y + 5);
//
//            sub3pointA = new Point(98,178);
//            sub3pointB = new Point(sub3pointA.x + 5, sub3pointA.y + 5);
//        }
    }

    @Override
    public Mat processFrame(Mat input)
    {

        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cb, 2);

        if(submat1 == null)
        {
            submat1 = Cb.submat(new Rect(sub1pointA, sub1pointB));
            submat2 = Cb.submat(new Rect(sub2pointA, sub2pointB));
            submat3 = Cb.submat(new Rect(sub3pointA, sub3pointB));
        }

        avg1 = (int) Core.mean(submat1).val[0];
        avg2 = (int) Core.mean(submat2).val[0];
        avg3 = (int) Core.mean(submat3).val[0];

        Imgproc.rectangle(
                input,
                sub1pointA,
                sub1pointB,
                new Scalar(0, 0, 255), 2);

        Imgproc.rectangle(
                input,
                sub2pointA,
                sub2pointB,
                new Scalar(0, 0, 255), 2);

        Imgproc.rectangle(
                input,
                sub3pointA,
                sub3pointB,
                new Scalar(0, 0, 255), 2);

        int maxOneTwo = Math.max(avg1, avg2);
        int max = Math.max(maxOneTwo, avg3);

        if(max == avg1)
        {
            Imgproc.circle(input, sub1pointA, 5, new Scalar(225, 52, 235), -1);
            position = SkystonePosition.LEFT;
        }
        else if(max == avg2)
        {
            Imgproc.circle(input, sub2pointA, 5, new Scalar(225, 52, 235), -1);
            position = SkystonePosition.CENTER;
        }
        else if(max == avg3)
        {
            Imgproc.circle(input, sub3pointA, 5, new Scalar(225, 52, 235), -1);
            position = SkystonePosition.RIGHT;
        }

        try
        {
            Thread.sleep(100);
        }
        catch (InterruptedException e)
        {
            e.printStackTrace();
        }

        return input;
    }

    public SkystonePosition getAnalysis()
    {
        return position;
    }
}