package ftc.teamcode.FreightFrenzy.Pipelines;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class ColorSignal extends OpenCvPipeline {

        int width = 35;
        int height = 40;

        Scalar rect = new Scalar(0, 0, 255);

        //green
        Scalar greenMin = new Scalar(30, 186, 50);
        Scalar greenMax = new Scalar(25, 237, 53);

        double green;
        double yes = 1;

        Mat greenSignal = new Mat();
        Mat greenMask = new Mat();
        Mat CutoutMat = new Mat();

        //purple
        Scalar purpleMin = new Scalar(145, 0, 180);
        Scalar purpleMax = new Scalar(200, 0, 255);

        Mat purpleSignal = new Mat();
        Mat getPurpleSignal = new Mat();

        Point topLeftPoint = new Point(150, 100);
        Point bottomRightPoint = new Point(topLeftPoint.x + width, topLeftPoint.y + height);

        Mat contoursOnPlainImageMat = new Mat();
        Scalar BLUE = new Scalar(0, 0, 255);
        static final int CONTOUR_LINE_THICKNESS = 2;

        @Override()
        public Mat processFrame(Mat input) {

                List<MatOfPoint> contoursList = new ArrayList<>();

                Imgproc.cvtColor(input, greenSignal, Imgproc.COLOR_RGB2HSV);
                Core.inRange(greenSignal, greenMin, greenMax, greenMask);
               // CutoutMat = greenSignal.submat(new Rect(topLeftPoint, bottomRightPoint));

                //green = (int) Core.mean(CutoutMat).val[0];

               // Imgproc.rectangle(input, topLeftPoint, bottomRightPoint, rect, 2);

               // if(green < yes) {
                //        Mat boo = new Mat();
               // }

                Imgproc.findContours(greenMask, contoursList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);


                input.copyTo(contoursOnPlainImageMat);
                Imgproc.drawContours(contoursOnPlainImageMat, contoursList, -1, BLUE, CONTOUR_LINE_THICKNESS, 8);




                return greenMask;
        }









}
