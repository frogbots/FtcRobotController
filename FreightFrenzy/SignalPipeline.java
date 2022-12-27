package ftc.teamcode.FreightFrenzy.Pipelines;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

class SignalPipeline extends OpenCvPipeline {

    static final int CB_CHAN = 1;
    static final int greenSigThresh = 100;
    static final int purpleSigThresh = 140;
    static final int pinkSigThresh = 180;
    static final int maxVal = 255;

    Scalar rect = new Scalar(0, 0, 255);


    Mat ycrcb = new Mat();
    Mat cb = new Mat();
    Mat sigThresh = new Mat();
    Mat region1_CB;


    int width = 10;
    int height = 15;
    int avg1;
    int signalNumber;

    Point topLeftPoint = new Point(110, 160);
    Point bottomRightPoint = new Point(topLeftPoint.x + width, topLeftPoint.y + height);



    @Override
    public Mat processFrame(Mat input) {

        Imgproc.cvtColor(input, ycrcb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(ycrcb, cb, CB_CHAN);
        Imgproc.threshold(cb, sigThresh, greenSigThresh, maxVal, Imgproc.THRESH_BINARY_INV);
        Imgproc.rectangle(input, topLeftPoint, bottomRightPoint, rect, 1);
        region1_CB = cb.submat(new Rect(topLeftPoint, bottomRightPoint));
        avg1 = (int) Core.mean(region1_CB).val[0];

        signalNumber = SignalNum();

        return input;
    }
    public int getAnalysis() {
        return avg1;
    }
    public int SignalNum() {

        int signalNumber = 0;

        if (avg1 > purpleSigThresh && avg1 < 180) {
            //160
            signalNumber = 3;
        } else if (avg1 > greenSigThresh && avg1 < 140) {
            //135
            signalNumber = 2;
        } else if (avg1 > pinkSigThresh) {
            signalNumber = 1;
        }
        return signalNumber;
    }
}