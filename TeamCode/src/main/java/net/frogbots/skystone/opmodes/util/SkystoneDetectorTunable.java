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

package net.frogbots.skystone.opmodes.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import net.frogbots.ftcopmodetunercommon.opmode.TunableLinearOpMode;

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

@TeleOp
public class SkystoneDetectorTunable extends TunableLinearOpMode
{
    OpenCvInternalCamera phoneCam;
    private volatile int ylw = 0;

    public SamplePipeline pipeline;

    @Override
    public void runOpMode()
    {
        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * In this sample, we're using the phone's internal camera. We pass it a
         * CameraDirection enum indicating whether to use the front or back facing
         * camera, as well as the view that we wish to use for camera monitor (on
         * the RC phone). If no camera monitor is desired, use the alternate
         * single-parameter constructor instead (commented out below)
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.FRONT, cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        //phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);

        /*
         * Open the connection to the camera device
         */
        phoneCam.openCameraDevice();

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        pipeline = new SamplePipeline();
        phoneCam.setPipeline(pipeline);

        /*
         * Tell the camera to start streaming images to us! Note that you must make sure
         * the resolution you specify is supported by the camera. If it is not, an exception
         * will be thrown.
         *
         * Also, we specify the rotation that the camera is used in. This is so that the image
         * from the camera sensor can be rotated such that it is always displayed with the image upright.
         * For a front facing camera, rotation is defined assuming the user is looking at the screen.
         * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
         * away from the user.
         */
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN, OpenCvInternalCamera.BufferMethod.SINGLE);

        /*
         * Wait for the user to press start on the Driver Station
         */
        waitForStart();

        while (opModeIsActive())
        {
            /*
             * Send some stats to the telemetry
             */
            telemetry.addData("Frame Count", phoneCam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", phoneCam.getFps()));
            telemetry.addData("Total frame time ms", phoneCam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", phoneCam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", phoneCam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", phoneCam.getCurrentPipelineMaxFps());
            telemetry.addData("avg1", pipeline.avg1);
            telemetry.addData("avg2", pipeline.avg2);
            telemetry.addData("avg3", pipeline.avg3);
            telemetry.addData("ylw", ylw);
            telemetry.update();

            /*
             * For the purposes of this sample, throttle ourselves to 10Hz loop to avoid burning
             * excess CPU cycles for no reason. (By default, telemetry is only sent to the DS at 4Hz
             * anyway). Of course in a real OpMode you will likely not want to do this.
             */
            sleep(100);
        }
    }

    /*
     * An example image processing pipeline to be run upon receipt of each frame from the camera.
     * Note that the processFrame() method is called serially from the frame worker thread -
     * that is, a new camera frame will not come in while you're still processing a previous one.
     * In other words, the processFrame() method will never be called multiple times simultaneously.
     *
     * However, the rendering of your processed image to the viewport is done in parallel to the
     * frame worker thread. That is, the amount of time it takes to render the image to the
     * viewport does NOT impact the amount of frames per second that your pipeline can process.
     *
     * IMPORTANT NOTE: this pipeline is NOT invoked on your OpMode thread. It is invoked on the
     * frame worker thread. This should not be a problem in the vast majority of cases. However,
     * if you're doing something weird where you do need it synchronized with your OpMode thread,
     * then you will need to account for that accordingly.
     */
    class SamplePipeline extends OpenCvPipeline
    {
        Mat submat1, submat2, submat3;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        Point sub1pointA = new Point(110,60);
        Point sub1pointB = new Point(sub1pointA.x + 5, sub1pointA.y + 5);

        Point sub2pointA = new Point(110,90);
        Point sub2pointB = new Point(sub2pointA.x + 5, sub2pointA.y + 5);

        Point sub3pointA = new Point(110,120);
        Point sub3pointB = new Point(sub3pointA.x + 5, sub3pointA.y + 5);

        int avg1, avg2, avg3;

        @Override
        public Mat processFrame(Mat input)
        {
            sub1pointA.x = getInt("x1");
            sub1pointA.y = getInt("y1");
            sub1pointB.x = sub1pointA.x + 5;
            sub1pointB.y = sub1pointA.y + 5;

            sub2pointA.x = getInt("x2");
            sub2pointA.y = getInt("y2");
            sub2pointB.x = sub2pointA.x + 5;
            sub2pointB.y = sub2pointA.y + 5;

            sub3pointA.x = getInt("x3");
            sub3pointA.y = getInt("y3");
            sub3pointB.x = sub3pointA.x + 5;
            sub3pointB.y = sub3pointA.y + 5;

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
            }
            else if(max == avg2)
            {
                Imgproc.circle(input, sub2pointA, 5, new Scalar(225, 52, 235), -1);
            }
            else if(max == avg3)
            {
                Imgproc.circle(input, sub3pointA, 5, new Scalar(225, 52, 235), -1);
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
    }
}