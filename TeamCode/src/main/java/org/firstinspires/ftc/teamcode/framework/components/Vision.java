package org.firstinspires.ftc.teamcode.framework.components;

import com.qualcomm.robotcore.util.ThreadPool;

import android.os.Handler;
import android.os.Looper;
import android.graphics.Bitmap;
import android.graphics.Color;
import com.vuforia.Frame;
import org.firstinspires.ftc.robotcore.external.function.*;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.function.Function;
import java.util.List;
import java.util.ArrayList;
import java.util.Arrays;

public class Vision {
    protected VuforiaLocalizer vuforia;

    public Vision(String webcam, HardwareMap hardwareMap) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        params.vuforiaLicenseKey = "AazZIlb/////AAABmRbCE9nGwUxMsIXmlS2x1+NMRNQ8Hz20HMiHWJeSBk9fXUYA5XnNqK6z4fAkSQmPHAxHfdp6DuLU6Qq1dVRe+sGvRuRPO15KyqgDIMqRAtlQQLOjyo0wuJF73BrtYGSWI9/axd7kUXLRBR9gurnTRqVxVLp8ktFsH05GoL4AR8fNP/UNJiEs/v7QQ5aBtYs4qhOGspKEV0YI/s+2ljKdWJpHcLRpu9jJYoFrbp47FZiRyK0L4VRWQ5dfxOUKyiCmQgID3j4ZHj0PGvwzz/c4n6OZxz7SrXW8pLPkfZE4H1+g6/bypvqRv8WZxrNgduI9IGGvIC5A+5IRqVcmqkTNIkIAgbAjV7mg/AeWx329RwF6";
        params.cameraName = hardwareMap.get(WebcamName.class, webcam);
        vuforia = ClassFactory.getInstance().createVuforia(params);
        vuforia.enableConvertFrameToBitmap();
    }

    public static class Pixels {
        public int width;
        public int height;
        
        /* [x+y*width][h|s|v] */
        public float[][] hsv;
    }

    public void readPixels (Function<Pixels, Void> cb) {
        vuforia.getFrameOnce(Continuation.create(ThreadPool.getDefault(), new Consumer<Frame>() {
            @Override public void accept(Frame frame) {
                Pixels pixels = new Pixels();

                Bitmap bitmap = vuforia.convertFrameToBitmap(frame);
                pixels.width = bitmap.getWidth();
                pixels.height = bitmap.getHeight();
                pixels.hsv = new float[pixels.width*pixels.height][3];

                for (int x = 0; x < pixels.width; x++) {
                    for (int y = 0; y < pixels.height; y++) {
                        int color = bitmap.getPixel(x, y);
                        float[] hsv = new float[3];
                        Color.colorToHSV(color, hsv);
                        pixels.hsv[x+y*pixels.width] = hsv;
                    }
                }

                cb.apply(pixels);
            }
        }));
    }

    public static class Result<T> {
        public T value;
        public String error;

        protected Result() {}
        public Result(T value) {
            this.value = value;
        }

        public static<T> Result<T> error(String error) {
            Result<T> result = new Result();
            result.error = error;
            return result;
        }
    }

    public static class Areas {
        public List<double[]> points;
        public int width;
        public int height;
        public List<int[]> colors;
    }

    public void getColorProfile(Function<Result<int[]>, Void> cb) {
        readPixels((data) -> {
            try {
                int xc = data.width / 2;
                int yc = data.height * 3 / 4;

                int xmin = xc - data.width / 8, xmax = xc + data.width / 8;
                int ymin = yc - data.height / 8, ymax = yc + data.height / 8;

                int[] ncolors = new int[9];

                for (int x = xmin; x < xmax; x++) {
                    for (int y = ymin; y < ymax; y++) {
                        int idx = x+y*data.width;
                        if (data.hsv[idx][1] > 0.65 && data.hsv[idx][2] >= 0.4) {
                            int colorIdx = (((int)(data.hsv[idx][0])+20)/40) % 9;
                            ncolors[colorIdx] += 1;   
                        }
                    }
                }

                cb.apply(new Result(ncolors));
            } catch(Exception e) {
                cb.apply(Result.error(e.getMessage()));
            }


            return null;
        });
    }
/*
    private void findAreas(Function<Result<Areas>, Void> cb) {
        readPixels((data) -> {
            final int TSIZE = 120;

            try {
                Areas areas = new Areas();
                areas.points = new ArrayList();
                areas.colors = new ArrayList();
                areas.width = data.width;
                areas.height = data.height;

                for (int lx = 0; lx < data.width-TSIZE; lx += TSIZE) {
                    for (int ly = 0; ly < data.height-TSIZE; ly += TSIZE) {
                        int ncolors[] = new int[9];

                        for (int dx = 0; dx < TSIZE; dx++) {
                            for (int dy = 0; dy < TSIZE; dy++) {
                                int x = lx+dx, y = lx+dy;
                                int idx = x+y*data.width;

                                if (idx > data.width*data.height) {
                                    continue;
                                }

                                // if saturation > 0.65
                                if (data.hsv[idx][1] > 0.65 && data.hsv[idx][1] >= 0.4) {
                                    // Hues divided into buckets
                                    // 340..20, 20..60, 60..100, 100..140, etc etc etc
                                    // (hue + 20) / 40
                                    int colorIdx = (((int)(data.hsv[idx][0])+20)/40) % 9;
                                    ncolors[colorIdx] += 1;
                                }
                            }
                        }

                        int n = 0;
                        for (int amt: ncolors) {
                            if (amt >= TSIZE*TSIZE/400)
                                n++;
                        }
                        if (n >= 4) {
                            areas.points.add(new double[]{(double)lx/areas.width + (double)TSIZE/areas.width/2, (double)ly/areas.height + + (double)TSIZE/areas.height/2});
                            areas.colors.add(ncolors);
                        }
                    }
                }

                cb.apply(new Result(areas));
            } catch(Throwable e) {
                cb.apply(Result.error(e.getMessage()));
            }

            return null;
        });
    }*/
} 