package ftc.teamcode.FreightFrenzy.Pipelines;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class Ycrcb extends OpenCvPipeline {


    Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(3, 3));
    Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(6, 6));


    Mat ycrcbCHAN2 = new Mat();
    Mat threshold = new Mat();
    Mat morphed = new Mat();
    Mat contoursOnPlainImageMat = new Mat();

    static final Scalar BLUE = new Scalar(0, 0, 255);

    static final int CONTOUR_LINE_THICKNESS = 2;

    enum Stage
    {
        FINAL,
        Cb,
        MASK,
        MASK_NR,
        CONTOURS
    }

    Ycrcb.Stage[] stages = Ycrcb.Stage.values();

    // Keep track of what stage the viewport is showing
    int stageNum = 0;

    @Override
    public void onViewportTapped()
    {
        /*
         * Note that this method is invoked from the UI thread
         * so whatever we do here, we must do quickly.
         */

        int nextStageNum = stageNum + 1;

        if(nextStageNum >= stages.length)
        {
            nextStageNum = 0;
        }

        stageNum = nextStageNum;
    }





    @Override
    public Mat processFrame(Mat input) {

        List<MatOfPoint> contoursList = new ArrayList<>();

        Imgproc.cvtColor(input, ycrcbCHAN2, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(ycrcbCHAN2, ycrcbCHAN2, 1);
        Imgproc.threshold(ycrcbCHAN2, threshold, 160, 255, Imgproc.THRESH_BINARY);
        morphMask(threshold, morphed);

        // Ok, now actually look for the contours! We only look for external contours.
        Imgproc.findContours(morphed, contoursList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

        // We do draw the contours we find, but not to the main input buffer.
        input.copyTo(contoursOnPlainImageMat);
        Imgproc.drawContours(contoursOnPlainImageMat, contoursList, -1, BLUE, CONTOUR_LINE_THICKNESS, 8);

        switch (stages[stageNum])

        {
            case Cb: {
                return ycrcbCHAN2;
            }

            case FINAL: {
                return input;
            }

            case MASK: {
                return threshold;
            }

            case MASK_NR: {
                return morphed;
            }

            case CONTOURS: {
                return contoursOnPlainImageMat;
            }
        }

       return input;
    }
    void morphMask(Mat input, Mat output)
    {
        /*
         * Apply some erosion and dilation for noise reduction
         */

        Imgproc.erode(input, output, erodeElement);
        Imgproc.erode(output, output, erodeElement);

        Imgproc.dilate(output, output, dilateElement);
        Imgproc.dilate(output, output, dilateElement);


    }
}