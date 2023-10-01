package org.firstinspires.ftc.team15091;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import java.util.Arrays;
import java.util.List;

public class RBProcessor implements VisionProcessor {

    public boolean add_sat_check = false;
    private static final Paint yellowPaint = new Paint();
    private static final Paint greenPaint = new Paint();

    String debug = "";
    Scalar[] means = new Scalar[3];
    public PixelPosition position = PixelPosition.Right;

    // Define region for left and middle
    static final List<Point> circleCenters = Arrays.asList(
            new Point(30, 315),
            new Point(360, 285),
            new Point(360, 400)
    );
    static final int radius = 25;
    public void init(int width, int height, CameraCalibration cameraCalibration) {
        yellowPaint.setColor(Color.YELLOW);
        yellowPaint.setStyle(Paint.Style.STROKE);
        yellowPaint.setStrokeWidth(2);
        yellowPaint.setTextSize(50);
        greenPaint.setColor(Color.GREEN);
        greenPaint.setStyle(Paint.Style.FILL);
    }

    public Object processFrame(Mat input, long captureTimeNanos) {
        return input;
    }

    private double getColorDifference (Scalar color, Scalar color2) {
        double dL = color2.val[0] - color.val[0];
        double da = color2.val[1] - color.val[1];
        double db = color2.val[2] - color.val[2];
        return Math.sqrt(dL * dL + da * da + db * db);
    }

    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object input) {
        Mat temp = new Mat();

        // Convert to RBG to HSV, so we can extract HUE channel
        Imgproc.cvtColor((Mat)input, temp, Imgproc.COLOR_RGBA2RGB);
        Imgproc.cvtColor(temp, temp, Imgproc.COLOR_RGB2Lab);

        for (int i = 0; i < 3; i++) {
            means[i] = calculateMeanHSVInCircle(temp, circleCenters.get(i), radius);
        }
        temp.release();
        double firstCircleDifference = getColorDifference(means[0], means[2]);
        double secondCircleDifference = getColorDifference(means[1], means[2]);
        if (firstCircleDifference > 30 && (!add_sat_check || Math.abs(means[0].val[1]) > 60 || Math.abs(means[0].val[2]) > 60)) {
            if (firstCircleDifference < secondCircleDifference) {
                canvas.drawCircle((float)circleCenters.get(1).x * scaleBmpPxToCanvasPx, (float)circleCenters.get(1).y * scaleBmpPxToCanvasPx, radius * scaleBmpPxToCanvasPx, greenPaint);
                position = PixelPosition.Middle;
            }
            else {
                position = PixelPosition.Left;
                canvas.drawCircle((float)circleCenters.get(0).x * scaleBmpPxToCanvasPx, (float)circleCenters.get(0).y * scaleBmpPxToCanvasPx, radius * scaleBmpPxToCanvasPx, greenPaint);
            }
        } else if (secondCircleDifference > 30 && (!add_sat_check || Math.abs(means[1].val[1]) > 60 || Math.abs(means[1].val[2]) > 60)) {
            position = PixelPosition.Middle;
            canvas.drawCircle((float)circleCenters.get(1).x * scaleBmpPxToCanvasPx, (float)circleCenters.get(1).y * scaleBmpPxToCanvasPx, radius * scaleBmpPxToCanvasPx, greenPaint);
        } else {
            position = PixelPosition.Right;
        }
        canvas.drawText(String.format("%.4f", getColorDifference(means[0], means[2])), 10, 60, yellowPaint);
        canvas.drawText(String.format("%.4f", getColorDifference(means[1], means[2])), 300, 60, yellowPaint);
        canvas.drawCircle((float)circleCenters.get(0).x * scaleBmpPxToCanvasPx, (float)circleCenters.get(0).y * scaleBmpPxToCanvasPx, radius * scaleBmpPxToCanvasPx, yellowPaint);
        canvas.drawCircle((float)circleCenters.get(1).x * scaleBmpPxToCanvasPx, (float)circleCenters.get(1).y * scaleBmpPxToCanvasPx, radius * scaleBmpPxToCanvasPx, yellowPaint);

        canvas.drawCircle((float)circleCenters.get(2).x * scaleBmpPxToCanvasPx, (float)circleCenters.get(2).y * scaleBmpPxToCanvasPx, radius * scaleBmpPxToCanvasPx, yellowPaint);
    }

    private static Scalar calculateMeanHSVInCircle(Mat image, Point center, int radius) {
        Mat mask = new Mat(image.size(), CvType.CV_8U, Scalar.all(0));
        Imgproc.circle(mask, center, radius, new Scalar(255), -1);  // Draw a filled circle on the mask
        Scalar meanColor = meanColorInCircle(image, mask);
        mask.release();  // Release resources
        return meanColor;
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