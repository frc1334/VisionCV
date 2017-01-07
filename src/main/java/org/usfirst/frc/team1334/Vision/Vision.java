package org.usfirst.frc.team1334.Vision;

import org.opencv.core.*;
import org.opencv.calib3d.*;
import org.opencv.imgcodecs.*;
import org.opencv.imgproc.*;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

/**
 * Created by FRC1334 on 9/26/2016.
 */

public class Vision {
    //if true, will write images to disk; DO NOT SET TRUE ON ROBOT
    boolean testing = false;
    public void setTesting(boolean testing){
        this.testing = testing;
    }
    // convert to Mat from 2 dimensional array of double
    static Mat Mat_from_array(double[][] in)
    {
        Mat out = new Mat(in.length, in[0].length, CvType.CV_64FC1);
        for(int i = 0; i < in.length; ++i)
            out.put(i, 0, in[i]);
        return out;
    }
    // convert to Mat of 3D point from 2 dimensional array of float
    MatOfPoint3f MatOfPoint3f_from_array(float[][] in)
    {
        Point3[] pts = new Point3[in.length];
        for(int i = 0; i < in.length; i++)
            pts[i] = new Point3(in[i][0], in[i][1], in[i][2]);
        return new MatOfPoint3f(pts);
    }
    //Stores information about an image to be used for testing vision algorithms
    class TestImage
    {
        String file_name;
        // top left, bottom left, bottom right, top right
        int[] corners;
        double dist, ofs, angle;
        public TestImage(String file_name, double ofs, double dist, double angle, int[] corners) {
            this.corners = corners;
            this.file_name = file_name;
            this.dist = dist;
            this.ofs = ofs;
            this.angle = angle;
        }
    }
    //selects an image for test_solvepnp from the array of test images
    int test_image_idx = 11;
    TestImage[] test_images;
    List<MatOfPoint> allcontours = new ArrayList<MatOfPoint>();
    List<MatOfPoint> filteredcontours = new LinkedList<MatOfPoint>();
    MatOfPoint targetContour;
    int contourthickness = Core.FILLED;
    Mat contourheirarchy;

    // thresholds image and extracts contour
    public void extractContours(){
        Mat imgHSV = img.clone();

        // HSV thresholding
        process.cvtColor(img, imgHSV, process.COLOR_BGR2HSV);
        Mat imgthresh = img.clone();
        //icore.inRange(imgHSV, new Scalar(60,69,124), new Scalar(88,255,255),imgthresh);
        //icore.inRange(imgHSV, new Scalar(57,64,80), new Scalar(91,255,255),imgthresh); // for angled
        //icore.inRange(imgHSV, new Scalar(34,64,78), new Scalar(89,255,255),imgthresh); // for 0,2
        icore.inRange(imgHSV, new Scalar(34,39,76), new Scalar(89,255,255),imgthresh);
        if(testing)
        codecs.imwrite("c:\\Users\\FRC1334\\Pictures\\Test\\binary.jpg",imgthresh);

        // contour extraction

        contourheirarchy = new Mat();
        //int approx = process.CHAIN_APPROX_SIMPLE;
        int approx = process.CHAIN_APPROX_NONE;
        process.findContours(imgthresh,allcontours,contourheirarchy,process.RETR_LIST, approx);

        if (testing){
            Mat imgContours = img_empty.clone();
            process.drawContours(imgContours,allcontours,-1,new Scalar(255,255,255),contourthickness,icore.LINE_8,contourheirarchy,2,new Point(0,0));
            codecs.imwrite("c:\\Users\\FRC1334\\Pictures\\Test\\contours1.jpg",imgContours);
        }

    }
    // filters out the target contour from noise contours
    public void filterContours(){
        Mat imgFilteredcontours = img_empty.clone();
        Mat imgFilteredcontours2 = img.clone();
        int i = 0;
        Mat imghull = img_empty.clone();
        int max_idx = 0;
        double maxarea = -1;
        //set the min and max y array sizes to the number of columns in the image
        int[] miny = new int[img.cols()];
        int[] maxy = new int[img.cols()];
        //iterate through each element in the arraylist of MatofPoint allcontours, and cast it to a MatofPoint c
        for(MatOfPoint c: allcontours) {
            // contour filtering
            double area = process.contourArea(c);
            MatOfPoint2f cF = new MatOfPoint2f(c.toArray());
            RotatedRect rect = process.minAreaRect(cF);

            double rectarea = rect.size.width * rect.size.height;
            // if the area of a contour is less than 100o px^2 or the area is less than half of the bounding box, treat it as noise
            if (area < 1000 || area / rectarea > 0.5) {
                i++;
                continue;
            }
            Rect boundingRect = process.boundingRect(c);
            //set all the minimum y of each column in the image to the height of the image, and the maximum to negative 1
            for(int j = boundingRect.x; j < boundingRect.x + boundingRect.width; j++) {
                miny[j] = img.rows();
                maxy[j] = -1;
            }
            // cpts is an array which stores the matrix of points in this contour
            Point[] cpts = c.toArray();
            // iterate through all of the points in this contour
            for(int j = 0; j < cpts.length; j++) {
                // point p is the j'th point in this contour
                Point p = cpts[j];
                // x is the x value of point p cast to integer from  a double, same for y
                int x = (int)p.x, y = (int)p.y;
                // if the y of this point is less than the current minimum y for this contour
                if(y < miny[x]) {
                    miny[x] = y;
                }
                if(y > maxy[x]) {
                    maxy[x] = y;
                }
            }
            int x1 = boundingRect.x + boundingRect.width / 3;
            int x2 = boundingRect.x + 2 * boundingRect.width / 3;
            int[] maxh = new int[] { -1,-1,-1};
            for(int j = boundingRect.x; j < boundingRect.x + boundingRect.width; j++) {
                int h = maxy[j] - miny[j];
                if(j > x2) {
                    if(h > maxh[2]) {
                        maxh[2] = h;
                    }
                } else if(j > x1) {
                    if(h > maxh[1]) {
                        maxh[1] = h;
                    }
                } else {
                    if(h > maxh[0]) {
                        maxh[0] = h;
                    }
                }
            }
            if(!(maxh[2]/maxh[1] >= 3 && maxh[0]/maxh[1]>=3)) {
                i++;
                continue;
            }
            process.drawContours(imgFilteredcontours, allcontours, i, new Scalar(255, 255, 255), contourthickness, icore.LINE_8, contourheirarchy, 2, new Point(0, 0));
            filteredcontours.add(c);
            i++;
            if(area>maxarea){
                maxarea = area;
                targetContour = c;
                max_idx = i-1;
            }
        }
        if(testing) {
            process.drawContours(imgFilteredcontours2, allcontours, max_idx, new Scalar(0, 0, 0), 1, icore.LINE_8, contourheirarchy, 2, new Point(0, 0));
            codecs.imwrite("c:\\Users\\FRC1334\\Pictures\\Test\\filtered_contours2.jpg", imgFilteredcontours2);
        }
    }
    // stores rough corners from hull lines at higher scope for use in other methods
    Point[] roughCorners = new Point[4];
    // extracts rough corners of target from target contour
    public void extractRoughCorners(){
        List<MatOfPoint> hullpts2 = new LinkedList<MatOfPoint>();

        //System.out.println(new Integer(lines.rows()).toString());

        Mat imghull = img_empty.clone();

        MatOfPoint c = targetContour;
        /// contour convex hull
        MatOfInt hull = new MatOfInt();
        process.convexHull(c,hull,true);
        Point[] hullpts = new Point[hull.rows()];
        Point[] cpts = c.toArray();
        int[] hpts = hull.toArray();
        for(int j = 0; j<hull.rows();j++){
            hullpts[j] = cpts[hpts[j]];
        }
        MatOfPoint hullpoints = new MatOfPoint(hullpts);
        hullpts2.add(hullpoints);
        process.drawContours(imghull,hullpts2,-1,new Scalar(255,255,255),contourthickness,icore.LINE_8,contourheirarchy,0,new Point(0,0));
        hullpts2.clear();

        /// contour hull line detection
        LineSegmentDetector LSD = process.createLineSegmentDetector();
        Mat imgGrayhull = new Mat();
        process.cvtColor(imghull,imgGrayhull,Imgproc.COLOR_BGR2GRAY);
        Mat lines = new Mat();
        LSD.detect(imgGrayhull,lines);
        if(testing) {
            Mat imgLines = img_empty.clone();
            LSD.drawSegments(imgLines, lines);
            codecs.imwrite("c:\\Users\\FRC1334\\Pictures\\Test\\hulllines.jpg", imgLines);
        }
        /// rough corner extraction
        Rect bound = process.boundingRect(hullpoints);
        float middlex = bound.width/2 + bound.x;

        Mat imgcorners = img.clone();
        //j=0 means left side, j=1 means right side
        for(int j = 0; j<2 ; j++){
            float miny = Float.MAX_VALUE;
            float maxy = -1;
            float minx = 0;
            float maxx = 0;
            for (int k = 0; k<lines.rows(); k++){
                float[] line = new float[4];
                lines.get(k,0,line);
                if((line[0] > middlex && j==0)||(line[0] < middlex && j==1))continue;
                if((line[2] > middlex && j==0)||(line[2] < middlex && j==1))continue;

                //process.circle(imgcorners,new Point((double)line[0],(double)line[1]),100,new Scalar(0,255,0),thickness,Imgproc.LINE_8,0);
                //process.circle(imgcorners,new Point((double)line[2],(double)line[3]),100,new Scalar(0,255,0),thickness,Imgproc.LINE_8,0);
                float dx = line[2] - line[0];
                float dy= line[3] - line[1];
                double angle = Math.atan2((double) dy,(double)dx) * 180/Math.PI + 180;
                //if (j==0)   System.out.println(new Double(angle).toString());
                double ae = 5;
                //roughly vertical +- 5 degrees
                if ((angle >= 90-ae && angle <=90 +ae)||(angle >= 270-ae && angle <= 270 + ae)){

                    for (int p = 0; p<2; p++){
                        float y = line[p*2+1];
                        float x = line[p*2];
                        if(y>maxy){
                            maxy = y;
                            maxx = x;
                        }
                        if(y<miny){
                            miny = y;
                            minx = x;
                        }
                    }
                }
            }
            if(j==0){
                roughCorners[0] = new Point(minx,miny);
                roughCorners[1] = new Point(maxx,maxy);
            }else{
                roughCorners[2] = new Point(maxx,maxy);
                roughCorners[3] = new Point(minx,miny);
            }
            if(testing) {
                process.circle(imgcorners, new Point((double) minx, (double) miny), 1, new Scalar(0, 0, 255), contourthickness, Imgproc.LINE_8, 0);
                process.circle(imgcorners, new Point((double) maxx, (double) maxy), 1, new Scalar(255, 0, 0), contourthickness, Imgproc.LINE_8, 0);
            }
        }

        if(testing) {
            codecs.imwrite("c:\\Users\\FRC1334\\Pictures\\Test\\roughcorners.jpg", imgcorners);
            codecs.imwrite("c:\\Users\\FRC1334\\Pictures\\Test\\hulls.jpg", imghull);
        }
    }
    // converts lines from point-vector form to an abc form for a formula to calculate intersection of lines
    double[] line_conv_pv_to_abc(double[] pv) {
        double vx = pv[0], vy = pv[1], x0 = pv[2], y0 = pv[3];
        double A = vy;
        double B = -vx;
        double C = A *x0 + B * y0;
        double[] abc = new double[]{ A,B,C};
        return abc;
    }
    // calculates the intersection point of 2 lines
    Point intersect(double A1, double B1, double C1, double A2, double B2, double C2) {
        /*print(A1, " ");
        print(B1, " ");
        print(C1, " ");
        print(A2, " ");
        print(B2, " ");
        print(C2, "\n");*/
        double det = A1*B2 - A2*B1;
        if(Math.abs(det) <= 1e-5){
            //Lines are parallel
            return new Point(0,0);
        }else{
            double x = (B2*C1 - B1*C2)/det;
            double y = (A1*C2 - A2*C1)/det;
            return new Point(x,y);
        }
    }
    //
    Point[] refinedCorners;
    // finds the refined coordinates of the corners based on the intersection of lines which are fit to the sides of the contour
    public void refineCorners(){
        Point[] closestPoints = new Point[4];
        int[] closestidx = new int[4];
        //System.out.println(targetContour.dump());
        Point[] contourPoints = targetContour.toArray();
        for(int i = 0; i<4; i++){
            double mindist = Float.MAX_VALUE;

            Point corner = roughCorners[i];
            Point closestPoint = null;
            for(int j = 0; j<contourPoints.length; j++){
                Point pt = contourPoints[j];
                double dx = pt.x - corner.x;
                double dy = pt.y - corner.y;
                double dist = Math.sqrt(dx*dx + dy*dy);
                if(dist<mindist){
                    mindist = dist;
                    closestPoint = pt;
                    closestidx[i] = j;
                }

            }
            closestPoints[i] = closestPoint;
        }

        double[][] lines = new double[4][];

        Mat imgRefinedLines = img.clone();
        for(int i = 0; i<4; i++){
            int nexti = (i+1)%4;
            int startidx = Math.min(closestidx[i], closestidx[nexti]);
            int endidx = Math.max(closestidx[i],closestidx[nexti]);
            int nrpts = (endidx-startidx+1);
            int nrpts_alt = contourPoints.length - endidx + startidx + 1;
            double minx = -1;
            double maxx = Float.MAX_VALUE;
            double minx1 = -1;
            double maxx1 = Float.MAX_VALUE;
            if (i==1){
                double dx= Math.abs(closestPoints[1].x - closestPoints[2].x);
                minx = closestPoints[1].x + dx * 0.05;
                maxx = closestPoints[2].x - dx * 0.05;
            }
            if(i == 3) {
                double dx= Math.abs(closestPoints[0].x - closestPoints[3].x);
                //double dy= Math.abs(closestPoints[0].y - closestPoints[3].y);
                double margin = dx * 0.02;
                double[] strip_x = new double[2];
                double middle_x = (closestPoints[0].x+closestPoints[3].x)/2;
                // k == 0 - left
                // k == 1 - right
                double third_y = closestPoints[3].y + (closestPoints[2].y - closestPoints[3].y)/3;
                int[] minxs = new int[img.rows()];
                int[] maxxs = new int[img.rows()];
                for(int k = 0; k < 2; k++) {

                    for(int j = (int)closestPoints[3].y; j <= third_y; j++) {
                        minxs[j] = img.cols();
                        maxxs[j] = -1;
                    }
                    for(int j = 0; j < contourPoints.length; j++) {
                        Point pt = contourPoints[j];
                        int x = (int)pt.x, y = (int)pt.y;
                        if(k == 0 && pt.x > middle_x) continue;
                        if(k == 1 && pt.x < middle_x) continue;
                        if(pt.y < closestPoints[3].y) continue;
                        if(pt.y > third_y) continue;
                        if(x < minxs[y])  {
                            minxs[y] = x;
                        }
                        if(x > maxxs[y]) {
                            maxxs[y] = x;
                        }
                    }
                    int total_w = 0;
                    int nr = 0;
                    for(int j = (int)closestPoints[3].y; j <= third_y; j++) {
                        int w = maxxs[j] - minxs[j];
                       // print(w, "\n");
                        total_w += w;
                        nr++;
                    }
                    double avg_w = total_w / (double)nr;
                    //print(avg_w, " avg_w\n");
                    if(k == 0) strip_x[k] = closestPoints[0].x + avg_w;
                    if(k == 1) strip_x[k] = closestPoints[3].x - avg_w;
                    /*
                    double yy = Math.max(closestPoints[0].y, closestPoints[3].y);
                    double xx = 0;
                    if (nrpts < nrpts_alt) {
                        for (int j = startidx; j <= endidx; j++) {
                            Point pt = contourPoints[j];
                            if(k == 0 && pt.x > middle_x) continue;
                            if(k == 1 && pt.x < middle_x) continue;
                            if (pt.y > yy) {
                                xx = pt.x;
                                break;
                            }
                        }
                    } else {
                        double miny = Double.MAX_VALUE;
                        for (int j = 0; j <= startidx; j++) {
                            Point pt = contourPoints[j];
                            if(k == 0 && pt.x > middle_x) continue;
                            if(k == 1 && pt.x < middle_x) continue;
                            if (pt.y > yy && pt.y < miny) {
                                miny = pt.y;
                                xx = pt.x;
                            }
                        }
                        for (int j = endidx; j < contourPoints.length; j++) {
                            Point pt = contourPoints[j];
                            if(k == 0 && pt.x > middle_x) continue;
                            if(k == 1 && pt.x < middle_x) continue;
                            if (pt.y > yy && pt.y < miny) {
                                miny = pt.y;
                                xx = pt.x;
                            }
                        }
                    }

                    strip_x[k] = xx;*/
                }
                //print(strip_x[0], " ");
                //print(strip_x[1], " sx\n");
                double[] strip_w = new double[]{strip_x[0] - closestPoints[0].x, closestPoints[3].x - strip_x[1]};
                minx = closestPoints[0].x + strip_w[0] * 0.2;
                maxx = strip_x[0] - strip_w[0] * 0.2;
                minx1 = strip_x[1] + strip_w[1] * 0.2;
                maxx1 = closestPoints[3].x - strip_w[1] * 0.2;
                /*print(minx, "minx\n");
                print(maxx, "maxx\n");
                print(minx1, "minx1\n");
                print(maxx1, "maxx1\n");*/
            }
            int k = 0;
            Mat sidePoints = null;
            if(nrpts < nrpts_alt) {
                sidePoints = new Mat(nrpts,2,CvType.CV_64FC1);
                for(int j = startidx; j<=endidx; j++){
                    Point pt = contourPoints[j];
                    //print(pt.x, " ");
                    //print(pt.y, "\n");
                    if(pt.x<minx||pt.x>maxx){
                        if(i != 3 || (pt.x<minx1||pt.x>maxx1))
                            continue;
                    }
                    double[] data = new double[2];
                    data[0] = pt.x;
                    data[1] = pt.y;
                    sidePoints.put(k++,0,data);
                }
            } else {
                //System.out.println("here!");
                nrpts = nrpts_alt;
                sidePoints = new Mat(nrpts,2,CvType.CV_64FC1);
                for(int j = 0; j<=startidx; j++){
                    Point pt = contourPoints[j];
                    //print(pt.x, " ");
                    //print(pt.y, "\n");
                    if(pt.x<minx||pt.x>maxx){
                        if(i != 3 || (pt.x<minx1||pt.x>maxx1))
                            continue;
                    }
                    double[] data = new double[2];
                    data[0] = pt.x;
                    data[1] = pt.y;
                    sidePoints.put(k++,0,data);
                }
                for(int j = endidx; j < contourPoints.length; j++){
                    Point pt = contourPoints[j];
                    //print(pt.x, " ");
                    //print(pt.y, "\n");
                    if(pt.x<minx||pt.x>maxx){
                        if(i != 3 || (pt.x<minx1||pt.x>maxx1))
                            continue;
                    }
                    double[] data = new double[2];
                    data[0] = pt.x;
                    data[1] = pt.y;
                    sidePoints.put(k++,0,data);
                }
            }
            sidePoints = sidePoints.rowRange(0, k-1);
            //System.out.println(sidePoints.dump());
            Mat lineMat = new Mat(1,4,CvType.CV_64FC1);
            process.fitLine(sidePoints,lineMat,process.CV_DIST_L2,0,0.01,0.01);
            float[] line = new float[4];
            lineMat.get(0,0,line);
            float y0 = line[3];
            float x0 = line[2];
            float vy = line[1];
            float vx = line[0];
            float xs = x0-(1000*vx);
            float ys = y0-1000*vy;
            float xe = x0+1000*vx;
            float ye = y0+1000*vy;
            /*print(x0," ");
            print(y0," ");
            print(vx," ");
            print(vy,"\n");*/
            if(testing) {
                Scalar[] colors = {new Scalar(0, 0, 255), new Scalar(255, 0, 0), new Scalar(0, 255, 0), new Scalar(255, 255, 0)};
                process.line(imgRefinedLines, new Point(xs, ys), new Point(xe, ye), colors[i], 0, Core.LINE_4, 0);
            }
            lines[i] = new double[] { vx,vy,x0,y0 };
        }
        if(testing)
        codecs.imwrite("c:\\Users\\FRC1334\\Pictures\\Test\\RefinedLines.jpg",imgRefinedLines);
        // intersect lines
        Mat imgcorners = img.clone();
        refinedCorners = new Point[4];
        for(int i = 0; i < 4; i++) {
            double[] L1 = line_conv_pv_to_abc(lines[i]);
            double[] L2 = line_conv_pv_to_abc(lines[(i + 1) % 4]);
            Point corner = intersect(L1[0], L1[1], L1[2], L2[0], L2[1], L2[2]);
            if(testing)
            process.circle(imgcorners, corner, 1, new Scalar(0, 0, 255), contourthickness, Imgproc.LINE_8, 0);
            refinedCorners[i] = corner;
        }
        if(testing) {
            codecs.imwrite("c:\\Users\\FRC1334\\Pictures\\Test\\corners.jpg", imgcorners);

            Mat imgTarget = img.clone();
            Scalar[] colors = {new Scalar(0, 0, 255), new Scalar(255, 0, 0), new Scalar(0, 255, 0), new Scalar(255, 255, 0)};
            for (int i = 0; i < 4; i++) {
                Point p1 = refinedCorners[i];
                Point p2 = refinedCorners[(i + 1) % 4];
                process.line(imgTarget, p1, p2, colors[i], 0, Core.LINE_4, 0);
            }
            codecs.imwrite("c:\\Users\\FRC1334\\Pictures\\Test\\target.jpg", imgTarget);
        }
    }

    // initializing the test image objects
    // each test image object has a filename, xoffset,zoffset, and imagecoordinate array
    public void init_test_images() {
        test_images = new TestImage[] {
                new TestImage("-0.5Mx, 1Mz.jpg", -0.5, 1, 0, new int[]{    // 0
                        162,330,
                        151,564,
                        487,573,
                        490,336
                }),
                new TestImage("1, +0.5.jpg", 0.5, 1, 0, new int[]{         // 1
                        794,341,
                        799,580,
                        1132,574,
                        1126,339
                }),
                new TestImage("1, 0.jpg", 0, 1, 0, new int[]{              // 2
                        473,336,
                        469,574,
                        810,577,
                        808,340
                }),
                new TestImage("2, +0.5.jpg", 0.5, 2, 0, new int[]{         // 3
                        747,359,
                        748,508,
                        962,507,
                        959,359
                }),
                new TestImage("2, +1.jpg", 1, 2, 0, new int[]{             // 4
                        953,359,
                        957,509,
                        1167,505,
                        1161,356
                }),
                new TestImage("2, -0.5.jpg", -0.5, 2, 0, new int[]{        // 5
                        337,355,
                        334,505,
                        548,506,
                        548,357
                }),
                new TestImage("2, 0.jpg", 0, 2, 0, new int[]{              // 6
                        544,358,
                        543,507,
                        757,507,
                        754,358
                }),
                new TestImage("2, -1.jpg", -1, 2, 0, new int[] {           // 7
                        152,352,
                        147,502,
                        362,504,
                        363,355
                }),
                new TestImage("3, +0.5.jpg", 0.5, 3, 0, new int[]{              // 8
                        726,366,
                        727,476,
                        884,475,
                        883,365
                }),
                new TestImage("3, +1.jpg", 1, 3, 0, new int[]{              // 9
                        877,366,
                        879,475,
                        1034,473,
                        1032,364
                }),
                new TestImage("3, -0.5.jpg", -0.5, 3, 0, new int[]{              // 10
                        421,363,
                        419,472,
                        576,472,
                        576,364
                }),
                new TestImage("3, 0.jpg", 0, 3, 0, new int[]{              // 11
                        576,365,
                        576,473,
                        731,473,
                        730,365
                }),
                new TestImage("3, -1.jpg", -1, 3, 0, new int[]{              // 12
                        372,360,
                        269,471,
                        427,472,
                        429,362
                }),
                new TestImage("1,0 angled.jpg", 0, 1, -45, new int[] {           //13
                        564, 346,
                        560, 560,
                        785, 603,
                        783, 333
                })
        };
    }

    String basepath = "c:\\Users\\FRC1334\\Pictures\\LED\\";

    float target_width = 50.8f;
    float target_height = 35.56f;
    float[][] target_points = new float[][]{
            {-target_width/2,-target_height,0},
            {-target_width/2,0,0},
            {target_width/2,0,0},
            {target_width/2,-target_height,0}
    };
    //draws lines between the points on images to check if they were entered in the correct order
    //lines should be red,blue,green,cyan going clockwise from the left side
    public void verify_test_images() {
        for(int i = 0; i < test_images.length; ++i) {
            TestImage timg = test_images[i];
            img = codecs.imread(basepath + timg.file_name);
            int[] c = timg.corners;
            Scalar[] colors = { new Scalar(0,0,255), new Scalar(255,0,0), new Scalar(0,255,0), new Scalar(255,255,0)};
            for(int j = 0; j < 4; j++) {
                int next_j = (j+1)%4;
                int ofs = j*2;
                int next_ofs = next_j*2;
                process.line(img, new Point(c[ofs+0],c[ofs+1]), new Point(c[next_ofs+0],c[next_ofs+1]), colors[j],3);
            }
            codecs.imwrite(basepath + new Integer(i).toString() + ".jpg", img);
        }
    }


    float camera_to_front_buffer_dist = 77; // cm
    float target_from_ground = 48.5f; // cm
    float camera_from_ground = 76; // cm
    float camera_to_center_buffer_axis = 13; // cm
    //calculates extrinsics for target
    public void calc_extrinsics() {


        // store the actual 3d world coordinates of the 4 corners for each target image in one array
        float[][] objpts = new float[test_images.length * 4][];
        for(int i = 0; i < test_images.length; ++i) {
            TestImage testImg = test_images[i];
            for(int j = 0; j < 4; j++) {
                float[] coords = new float[3];
                for(int k = 0; k < 3; k++)
                    coords[k] = target_points[j][k];
                coords[0] += testImg.ofs; // add the X offset of the target
                coords[1] -= target_from_ground; // y is down, so subtract
                coords[2] += testImg.dist; // add the distance to the target
                objpts[i*4+j] = coords;
            }
        }
        MatOfPoint3f objectPoints = MatOfPoint3f_from_array(objpts);

        Mat rvec = new Mat();
        Mat tvec = new Mat();

        // store the image coordinates of the 4 corners from each target image into one array
        Point imgPoints[] = new Point[test_images.length * 4];
        for(int i = 0; i < test_images.length; ++i) {
            TestImage timg = test_images[i];
            for (int j = 0; j < 4; j++) {
                imgPoints[i*4+j] = new Point((double) timg.corners[j * 2], (double) timg.corners[j * 2 + 1]);
            }
        }

        MatOfPoint2f imagePoints = new MatOfPoint2f(imgPoints);

        //System.out.println(objectPoints.dump());
        System.out.println(imagePoints.dump());

        boolean useExtrinsicGuess = true;
        int flags = calib.CV_ITERATIVE;

        if(useExtrinsicGuess) {
            double[] tvec_hint = new double[] { -camera_to_center_buffer_axis, -camera_from_ground, camera_to_front_buffer_dist };
            double a = (11.126855419068251) / 180 * Math.PI;
            double[] rot_hint = new double[] {
                    1,0,0,
                    0, Math.cos(a), -Math.sin(a),
                    0, Math.sin(a), Math.cos(a) };
            Mat rmat = new Mat(3,3,CvType.CV_64FC1);
            rmat.put(0,0,rot_hint);
            //System.out.println(rmat.dump());
            rvec = new Mat(3,1,CvType.CV_64FC1);
            calib.Rodrigues(rmat, rvec);
            //System.out.println(rvec.dump());
            tvec = new Mat(3,1,CvType.CV_64FC1);
            tvec.put(0,0,tvec_hint);
        }

        calib.solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec, useExtrinsicGuess, flags);
        //Mat inliers = new Mat();
        //calib.solvePnPRansac(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec, useExtrinsicGuess, 100, 1.0f, 0.99f, inliers, flags);

        MatOfPoint2f imagePoints2 = new MatOfPoint2f();
        calib.projectPoints(objectPoints, rvec, tvec, cameraMatrix, distCoeffs, imagePoints2);
        Point imgPoints2[] = imagePoints2.toArray();
        double total_dist = 0;
        for(int i = 0; i < test_images.length * 4; i++) {
            Point p = imgPoints2[i];
            //TestImage timg = test_images[i/4];
            Point q = imgPoints[i];//new Point(timg.corners[(i%4)*2+0], timg.corners[(i%4)*2+1]);
            double dx = p.x-q.x, dy = p.y-q.y;
            double dist = Math.sqrt(dx*dx+dy*dy);
            total_dist += dist;
            //System.out.println(new Double(p.x).toString() + ", " + new Double(p.y).toString());
        }
        total_dist /= test_images.length * 4;
        print(total_dist, " average dist\n");

        /*System.out.print("rvec: " );
        for(int i = 0; i < 3; i++) {
            double data[] = new double[1];
            rvec.get(i,0, data);
            System.out.print(new Double(data[0]).toString() + ", ");
        }
        System.out.println("");*/
        double dist = 0;
        System.out.print("tvec: " );
        for(int i = 0; i < 3; i++) {
            double data[] = new double[1];
            tvec.get(i,0, data);
            dist += data[0] * data[0];
            System.out.print(new Double(data[0]).toString() + ", ");
        }
        System.out.println("");
        dist = Math.sqrt(dist);
        System.out.println(new Double(dist).toString());
    }

    Mat transWtoC;
    Mat rotWtoCinv;
    // inits extrinsics from camera calibration results
    public void init_extrinsics()
    {
        double rvec[] = {0.2113553805307846, 0.0360187554417961, -0.0017223450881762232};
        double tvec[] = {2.970115271755897, 83.61113846034421, 226.80230321473837};
        Mat rvecMat = new Mat(3, 1, CvType.CV_64F);
        transWtoC = rvecMat.clone();
        for(int i = 0; i < 3; i++) {
            double[] data = new double[1];
            data[0] = rvec[i];
            rvecMat.put(i, 0, data);
            data[0] = tvec[i];
            transWtoC.put(i,0,data);
        }
        Mat rotWtoC = new Mat(3,3, CvType.CV_64F);
        calib.Rodrigues(rvecMat, rotWtoC);
        rotWtoCinv = rotWtoC.clone();
        icore.transpose(rotWtoC, rotWtoCinv);
    }
    // print methods for different number variables to strings (with tail strings)
    public void print(double val) {
        System.out.print(new Double(val).toString());
    }

    public void print(double val, String tail) {
        print(val);
        System.out.print(tail);
    }

    public void print(float val) {
        System.out.print(new Float(val).toString());
    }

    public void print(float val, String tail) {
        print(val);
        System.out.print(tail);
    }
    public void print(int val) {
        System.out.print(new Integer(val).toString());
    }
    public void print(int val, String tail) {
        print(val);
        System.out.print(tail);
    }
    //uses solvepnp on the test images at index test_image_idx
    public void test_solvePnP() {
        TestImage timg = test_images[test_image_idx];
        for(int i = 0; i < 4; i++) {
            roughCorners[i] = new Point( (double)timg.corners[i*2], (double)timg.corners[i*2+1]);
        }
        solvePnP();
    }
    // solves for the distance of a target which is directly in front of the camera
    public void solvePnP() {

        float width = 50.8f;
        float height = 35.56f;
        MatOfPoint3f objectPoints = MatOfPoint3f_from_array(target_points);

        Mat rvec = new Mat();
        Mat tvec = new Mat();

        MatOfPoint2f imagePoints = new MatOfPoint2f(roughCorners);

        boolean useExtrinsicGuess = false;
        //int flags = calib.CV_P3P;
        int flags = calib.CV_ITERATIVE;
        calib.solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec, useExtrinsicGuess, flags);

        /*System.out.print("rvec: " );
        for(int i = 0; i < 3; i++) {
            double data[] = new double[1];
            rvec.get(i,0, data);
            System.out.print(new Double(data[0]).toString() + " ");
        }
        System.out.println("");*/
        double dist = 0;
        System.out.print("tvec: " );
        for(int i = 0; i < 3; i++) {
            double data[] = new double[1];
            tvec.get(i,0, data);
            dist += data[0] * data[0];
            print(data[0]);
            System.out.print(" ");
        }
        System.out.println("");
        dist = Math.sqrt(dist);
        System.out.println(new Double(dist).toString());

        Mat rotOtoC = new Mat(3, 3, CvType.CV_64F);
        calib.Rodrigues(rvec, rotOtoC);
        Mat transOtoC = tvec;

        boolean use_extrinsics = false;
        if(use_extrinsics) {
            Mat rotOtoW = rotOtoC.clone();
            Mat transOtoW = tvec.clone();
            System.out.println(new Integer(tvec.rows()).toString() + " " + new Integer(tvec.cols()).toString());
            icore.gemm(rotWtoCinv, rotOtoC, 1.0, tvec, 0.0, rotOtoW);
            Mat rwcinvxtwc = tvec.clone();
            icore.gemm(rotWtoCinv, transWtoC, 1.0, tvec, 0.0, rwcinvxtwc);
            icore.gemm(rotWtoCinv, transOtoC, 1.0, rwcinvxtwc, -1.0, transOtoW);

            dist = 0;
            System.out.print("transOtoW: ");
            for (int i = 0; i < 3; i++) {
                double data[] = new double[1];
                transOtoW.get(i, 0, data);
                dist += data[0] * data[0];
                print(data[0], " ");
            }
            System.out.println("");
            dist = Math.sqrt(dist);
            System.out.println(new Double(dist).toString());
        } else {
            Mat[] PC = new Mat[4];
            double[][] PCv = new double[4][];
            for(int i = 0; i < 4; i++) {
                float[] p = target_points[i];
                // compute camera coordinates of object points: PC = R * P + T
                Mat P = tvec.clone();
                double [] p1 = {p[0],p[1],p[2]};
                P.put(0,0,p1);
                PC[i] = tvec.clone();
                icore.gemm(rotOtoC, P, 1.0, transOtoC, 1.0, PC[i]);
                //System.out.println(PC[i].dump());
                PCv[i] = new double[3];
                PC[i].get(0,0,PCv[i]);
            }
            Mat T12 = tvec.clone();
            icore.subtract(PC[1], PC[0], T12);
            double lengthT12 = icore.norm(T12);
            Mat TA = tvec.clone();
            icore.multiply(T12, new Scalar(target_from_ground / lengthT12), TA);
            Mat A = tvec.clone();
            icore.add(tvec, TA, A);
            dist = icore.norm(TA.cross(tvec)) / icore.norm(TA) - camera_to_front_buffer_dist;
            print(dist, "\n");

            double rotY = Math.atan2(PCv[2][2] - PCv[1][2], PCv[2][0] - PCv[1][0]) / Math.PI * 180;
            print(rotY, " rotY\n");
            double rotZ = Math.atan2(PCv[2][1] - PCv[1][1], PCv[2][0] - PCv[1][0]) / Math.PI * 180;
            print(rotZ, " rotZ\n");
            double rotX = Math.atan2(PCv[1][2] - PCv[0][2], PCv[1][1] - PCv[0][1]) / Math.PI * 180;
            print(rotX, " rotX\n");

            /*double[] tvec1 = new double[3];
            tvec.get(0,0,tvec1);
            double rot = Math.atan2(tvec1[1],tvec1[2]) * 180 / Math.PI;// + 180;
            print(rot, " rot\n");*/
        }
        if(testing) {
            Mat imgReprojected = img.clone();
            MatOfPoint2f imagePoints2 = new MatOfPoint2f();
            calib.projectPoints(objectPoints, rvec, tvec, cameraMatrix, distCoeffs, imagePoints2);
            Point[] imgPoints2 = imagePoints2.toArray();
            for (int i = 0; i < imgPoints2.length; ++i) {
                process.line(imgReprojected, imgPoints2[i], imgPoints2[(i + 1) % imgPoints2.length], new Scalar(0, 0, 0), 0, Core.LINE_4, 0);
            }
            codecs.imwrite("c:\\Users\\FRC1334\\Pictures\\Test\\reprojected.jpg", imgReprojected);
        }
    }

    // opencv stuff
    Core icore = new Core();
    Imgproc process = new Imgproc();
    Imgcodecs codecs = new Imgcodecs();
    Calib3d calib = new Calib3d();

    /// original image
    Mat origimg;
    Mat img_empty;
    Mat img;
    // initializing opencv
    public void opencv_init() {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
    }
    // opens image file for test images
    public void open_image() {
        //origimg = codecs.imread("c:\\Users\\FRC1334\\Pictures\\LED\\1, +0.5.jpg");
        TestImage ti = test_images[test_image_idx];
        if(ti.angle == 0)
            origimg = codecs.imread("c:\\Users\\FRC1334\\Pictures\\LED\\" + ti.file_name);
        else
            origimg = codecs.imread("c:\\Users\\FRC1334\\Pictures\\Camera Roll\\" + ti.file_name);
        set_image(origimg);
    }

    public void set_image(Mat image) {
        origimg = image;
        img_empty = origimg.clone();
        img_empty.setTo(new Scalar(0,0,0));
        img = img_empty.clone();
        img = origimg;
    }

    Mat cameraMatrix;
    MatOfDouble distCoeffs;
    // initializing camera parameters from calibration
    public void init_params() {
        cameraMatrix = Mat_from_array( new double[][]{
                {1.1721082270179563e+03, 0., 6.3277026007692382e+02},
                { 0., 1.1721082270179563e+03, 3.4160864142440721e+02},
                {0., 0., 1.}
        });

        double[] distCs = new double[] {
                1.8417087639556795e-01, -1.3483129339487023e+00,
                1.9206696553095034e-04, 6.0641995000998449e-04,
                2.7214836109825287e+00
        };
        Mat distC = new Mat(5, 1, CvType.CV_64FC1);
        for(int i = 0; i < 5; i++) {
            double[] data = new double[1];
            data[0] = distCs[i];
            distC.put(i,0,data);
        }

        distCoeffs = new MatOfDouble(distC );
        /*for(int i = 0; i < 5; i++) {
            double[] data = new double[1];
            distC.get(i,0,data);
            System.out.println(new Double(data[0]).toString());
        }*/

        /// test undistort
        //img = origimg.clone();
        //process.undistort(origimg,img,cameraMatrix,distCoeffs);
        //codecs.imwrite("c:\\Users\\FRC1334\\Pictures\\Test\\undistorted.jpg",img);
    }
}
