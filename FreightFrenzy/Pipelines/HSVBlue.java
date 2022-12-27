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


public class HSVBlue extends OpenCvPipeline {





        static final int CONTOUR_LINE_THICKNESS = 2;

        Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(2, 2));
        Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(5, 5));

        Scalar min = new Scalar(110, 70, 50);
        Scalar max = new Scalar(130, 255, 255);


        final static Scalar BLUE = new Scalar(0, 0, 255);

        Mat imageHSV = new Mat();
        Mat mask1 = new Mat();
        Mat Morphed = new Mat();
        Mat contoursOnPlainImageMat = new Mat();


        enum Stage
        {
                FINAL,
                Cb,
                MASK,
                MASK_NR,
                CONTOURS
        }

        HSVBlue.Stage[] stages = HSVBlue.Stage.values();

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

                Imgproc.cvtColor(input, imageHSV, Imgproc.COLOR_RGB2HSV);
                Core.inRange(imageHSV, min, max, mask1);
                morphMask(mask1, Morphed);


                Imgproc.findContours(Morphed, contoursList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);


                input.copyTo(contoursOnPlainImageMat);
                Imgproc.drawContours(contoursOnPlainImageMat, contoursList, -1, BLUE, CONTOUR_LINE_THICKNESS, 8);




                switch (stages[stageNum])

                {
                        case Cb: {
                                return imageHSV;
                        }

                        case FINAL: {
                                return input;
                        }

                        case MASK: {
                                return mask1;
                        }

                        case MASK_NR: {
                                return Morphed;
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



