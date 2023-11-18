package org.firstinspires.ftc.team15091;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import java.util.Arrays;
import java.util.List;

public class RBPipeline extends PipelineBase {
    String debug = "";
    Double[] hues = new Double[2];
    PixelPosition position = PixelPosition.Right;

    // Define region for left and middle
    static final List<Rect> rectRegions = Arrays.asList(
            new Rect(0, 190, 60, 50),
            new Rect(320, 165, 60, 50)
    );
    static final List<Point> circleCenters = Arrays.asList(
            new Point(30, 215),
            new Point(350, 185)
    );
    static final int radius = 25;

    @Override
    public Mat processFrame(Mat input) {
        Mat temp = new Mat();

        // Convert to RBG to HSV, so we can extract HUE channel
        Imgproc.cvtColor(input, temp, Imgproc.COLOR_RGBA2RGB);
        Imgproc.cvtColor(temp, temp, Imgproc.COLOR_RGB2HSV);

        for (int i = 0; i < 2; i++) {
            hues[i] = calculateMeanHueInCircle(temp, circleCenters.get(i), radius);
            Imgproc.circle(input, circleCenters.get(i), radius, YELLOW, 2);
        }
        temp.release();

        if (isRed(hues[0]) || isBlue(hues[0])) {
            position = PixelPosition.Left;
            Imgproc.circle(input, circleCenters.get(0), radius, GREEN, -1);
        } else if (isRed(hues[1]) || isBlue(hues[1])) {
            position = PixelPosition.Middle;
            Imgproc.circle(input, circleCenters.get(1), radius, GREEN, -1);
        } else {
            position = PixelPosition.Right;
        }
        debug = String.format(" %3.0f, %3.0f", hues[0], hues[1]);

        return input;
    }

    boolean isRed(Double hueValue) {
        return hueValue > 0 && hueValue < 20;
    }

    boolean isBlue(Double hueValue) {
        return hueValue > 99 && hueValue < 120;
    }

    private static Double calculateMeanHueInCircle(Mat image, Point center, int radius) {
        Mat mask = new Mat(image.size(), CvType.CV_8U, Scalar.all(0));
        Imgproc.circle(mask, center, radius, new Scalar(255), -1);  // Draw a filled circle on the mask
        Scalar meanColor = meanColorInCircle(image, mask);
        mask.release();  // Release resources
        return meanColor.val[0];
    }

    private static Scalar meanColorInCircle(Mat image, Mat mask) {
        // Apply the mask to the image
        Mat maskedImage = new Mat();
        Core.bitwise_and(image, image, maskedImage, mask);

        // Calculate the mean color using Core.mean
        Scalar meanColor = Core.mean(maskedImage, mask);

        // Release resources
        maskedImage.release();

        return meanColor;
    }
}
