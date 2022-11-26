package ftc.teamcode.FreightFrenzy.Pipelines;


 import org.opencv.core.Mat;
 import org.opencv.core.Rect;
 import org.opencv.core.Scalar;
 import org.opencv.imgproc.Imgproc;
 import org.openftc.easyopencv.OpenCvPipeline;

         public class PipelineAttempt extends OpenCvPipeline {


             public Scalar nonSelectedColor = new Scalar(0, 255, 0);
             public Scalar selectedColor = new Scalar(0, 0, 255);


             public Rect rect2 = new Rect(160, 42, 40, 40);


             public int selectedRect = 0;


             Mat hsvMat = new Mat();

             @Override
             public Mat processFrame(Mat input) {


                 return input;
             }


         }
