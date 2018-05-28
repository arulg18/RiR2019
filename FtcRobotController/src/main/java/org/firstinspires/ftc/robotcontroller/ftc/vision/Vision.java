package org.firstinspires.ftc.robotcontroller.ftc.vision;

import android.view.SurfaceView;

import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.Mat;

public class Vision {
    public static class FrameGrabber implements CameraBridgeViewBase.CvCameraViewListener2 {

        @Override
        public void onCameraViewStarted(int width, int height) {

        }

        @Override
        public void onCameraViewStopped() {

        }

        @Override
        public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
            return inputFrame.rgba();
        }
        public FrameGrabber(CameraBridgeViewBase c) {
            c.setVisibility(SurfaceView.VISIBLE);
            c.setCvCameraViewListener(this);
        }
    }
}
