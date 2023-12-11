package org.firstinspires.ftc.teamcode.drive;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class FindRegionPipeline extends OpenCvPipeline {

    private Mat YCrCb = new Mat();
    private Mat leftCrop;
    private Mat rightCrop;
    private double leftAvgFinal;
    private double rightAvgFinal;
    private Mat output = new Mat();
    private Scalar rectColor = new Scalar(255.0, 0.0, 0.0);
    private Side color = null;

    public FindRegionPipeline(Side color) {
        this.color = color;
    }

    public Mat processFrame(Mat input) {

        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);

        Rect leftRect = new Rect(60, 70, 155, 195); // 165 160 105 140
        Rect rightRect = new Rect(770, 40, 155, 155); // 370 160 105 95

        input.copyTo(output);
        Imgproc.rectangle(output, leftRect, rectColor, 2);
        Imgproc.rectangle(output, rightRect, rectColor, 2);

        leftCrop = YCrCb.submat(leftRect);
        rightCrop = YCrCb.submat(rightRect);

        Core.extractChannel(leftCrop, leftCrop, Globals.COLOR == Side.BLUE ? 2 : 1);
        Core.extractChannel(rightCrop, rightCrop, Globals.COLOR == Side.BLUE ? 2 : 1);

        Scalar leftAvg = Core.mean(leftCrop);
        Scalar rightAvg = Core.mean(rightCrop);

        leftAvgFinal = leftAvg.val[0];
        rightAvgFinal = rightAvg.val[0];



        return output;
    }

    public double getLeftAvgFinal() {
        return this.leftAvgFinal;
    }

    public Side getLocation() {
        if (leftAvgFinal - rightAvgFinal > 7) {
            return Side.LEFT;
        } else if (rightAvgFinal - leftAvgFinal >5) {
            return Side.CENTER;
        } else {
            return Side.RIGHT;
        }
    }

    public double getRightAvgFinal() {
        return this.rightAvgFinal;
    }
}