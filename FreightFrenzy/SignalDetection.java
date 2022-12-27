/*
 * Copyright (c) 2019 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package ftc.teamcode.FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

@Disabled
@TeleOp
public class SignalDetection extends LinearOpMode {
    OpenCvCamera phoneCam;

    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.FRONT, cameraMonitorViewId);


      SignalPipeline2 pipeline = new SignalPipeline2();
        phoneCam.setPipeline(pipeline);


        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {

                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();


        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("Frame Count", phoneCam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", phoneCam.getFps()));
            telemetry.addData("Total frame time ms", phoneCam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", phoneCam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", phoneCam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", phoneCam.getCurrentPipelineMaxFps());
            telemetry.addData("thresh value", pipeline.getAnalysis());
            telemetry.addData("signalNumber", pipeline.signalNumber);
            telemetry.update();


            if (gamepad1.a) {

                phoneCam.stopStreaming();
                //phoneCam.closeCameraDevice();
            }


            sleep(100);
        }
    }


    class SignalPipeline2 extends OpenCvPipeline {

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
}