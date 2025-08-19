// Code is heavily inspired by https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/intermediatevision/Robot.java
package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Most of the code here will run in a separate thread.
 */
public class VisionSubsystem extends SubsystemBase {
    
    public VisionSubsystem() {

        // Starts a new thread because vision is computationally expensive
        Thread visionThread = new Thread(
            () -> {
                // VISION CODE GOES INSIDE HERE
                // Get the UsbCamera from CameraServer (camera must be plugged into the RoboRIO)
                UsbCamera camera = CameraServer.startAutomaticCapture();

                // Set the resolution (these can be any arbitrary values)
                camera.setResolution(Constants.RESOLUTION_X, Constants.RESOLUTION_Y);

                // The CvSink will capture Mats from the camera (capture frames).
                CvSink cvSink = CameraServer.getVideo();
                // The CvSource will send images back to the Dashboard.
                CvSource outputStream = CameraServer.putVideo("Vision", Constants.RESOLUTION_X, Constants.RESOLUTION_Y);

                // Mats are very memory expensive. Lets reuse this Mat.
                Mat raw = new Mat();
                Mat processed = new Mat();
                Mat hierarchy = new Mat();

                // Stores all the objects in the scene
                List<MatOfPoint> contours = new ArrayList<>();

                // This cannot be 'true'. The program will never exit if it is. This
                // lets the robot stop this thread when restarting robot code or
                // deploying.
                while (!Thread.interrupted()) {
                    // Tell the CvSink to grab a frame from the camera and put it in the source mat.
                    // If there is an error (== 0) notify the output.
                    if (cvSink.grabFrame(raw) == 0) {
                        // Send the error to the output stream (Dashboard)
                        outputStream.notifyError(cvSink.getError());
                        // skip the rest of the current iteration
                        continue;
                    }



                    // IMAGE PROCESSING
                    // Step 1) Blur the image to get rid of noise.
                    Imgproc.blur(raw, processed, new Size(7,7));
                    // Step 2) Convert the frame to HSV for thresholding
                    Imgproc.cvtColor(processed, processed, Imgproc.COLOR_BGR2HSV);
                    // Step 3) Use the threshold values from the shuffleboard to mask out certain colors.
                    Core.inRange(
                        processed, 
                        new Scalar(
                            0,
                            0,
                            0
                        ), 
                        new Scalar(
                            150,
                            150,
                            150
                        ),
                        processed
                    );
                    // Step 4) Find the objects based upon the mask (by finding contours [edges and shapes])
                    Imgproc.findContours(processed, contours, hierarchy, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);

                    // if any contour exist...
                    if (hierarchy.size().height > 0 && hierarchy.size().width > 0)
                    {
                        // for each contour, display it in blue
                        for (int idx = 0; idx >= 0; idx = (int) hierarchy.get(0, idx)[0])
                        {
                            Imgproc.drawContours(raw, contours, idx, new Scalar(250, 0, 0));
                        }
                    }
                    // Give the output stream a new image to display
                    if (false)
                        outputStream.putFrame(raw);
                    else
                        outputStream.putFrame(processed);
                    contours.clear();
                }
            }
        );

        // Start the thread.
        visionThread.setDaemon(true);
        visionThread.start();
    }

}
