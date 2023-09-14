/*
Malhar Mahant & Kruthika Gangaraju

CS 5330 Computer Vision
Spring 2023

CPP program for detecting, extracting corners from patterns in image, saving them to file, and using saved calibration parameters to display virtual objects in the scene.

*/

#define _CRT_SECURE_NO_WARNINGS
#include <direct.h>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <dirent.h>
#include <iostream>
#include <windows.h>
#include <commdlg.h>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include "csv_util.h"
#include "CalibrationFunction.hpp"
#include "Main.h"

cv::Mat canvas;
cv::Mat displayImage;
std::string windowName = "Calibration and Augmented Reality";
// Current step
char key = 0;
char temp = key;

/*
* Helper method to print the given matrix to the console
*/
void printMatrix(cv::Mat& matrix) {
    for (int i = 0; i < matrix.rows; i++) {
        for (int j = 0; j < matrix.cols; j++) {
            cout << matrix.at<double>(i, j) << "\t";
        }
        cout << endl;
    }
}

/* 
* Helper method to extract corners of a chessboard or asymmetric circles grid pattern from the image.
*/
void extractCornersFromImage(cv::Mat& detectedCorner, cv::Mat& frame, cv::Size& patternSize, std::vector<cv::Point2f>& cornerSet)
{
    extract_corners(frame, detectedCorner, patternSize, cornerSet);
    detectedCorner.copyTo(displayImage);
}

int main(int argc, char* argv[])
{
    cv::VideoCapture* capdev;

    // open the video device
    capdev = new cv::VideoCapture(1);
    if (!capdev->isOpened()) {
        printf("Unable to open video device\n");
        return(-1);
    }

    // get some properties of the image
    cv::Size refS((int)capdev->get(cv::CAP_PROP_FRAME_WIDTH),
        (int)capdev->get(cv::CAP_PROP_FRAME_HEIGHT));
    printf("Expected size: %d %d\n", refS.width, refS.height);

    cv::namedWindow(windowName, 1); // identifies a window
    cv::Mat frame;
    cv::Mat detectedCorner;
    cv::Size detectedPatternSize(9, 6);
    std::vector<cv::Point2f> cornerSet;
    std::vector<std::vector<cv::Point2f>> corners_list;
    std::vector<cv::Vec3f> pointSet;
    std::vector<std::vector<cv::Vec3f>> points_list;
    int frameno = 1;
    int saveCount = 1;
    cv::VideoWriter videoWriter = cv::VideoWriter();
    cv::Mat camera_matrix;
    float reproj_err = 0.0;
    cv::Mat dist_coeff;
    cv::Mat camera_matrix_read;
    cv::Mat dis_coef_read;
    cv::Mat rotate_vec;
    cv::Mat tran_vec;
    bool artworkControl = false;
    bool is3daxes = false;
    bool isvirtualobject = false;
    key = 0;
    while (cv::getWindowProperty(windowName, 0) >= 0) {

        *capdev >> frame; // get a new frame from the camera, treat as a stream
        if (frame.empty()) {
            printf("frame is empty\n");
            break;
        }

        if (key == 0)
            frame.copyTo(displayImage);

        // Task 1
        if (key == '1') {
            extractCornersFromImage(detectedCorner, frame, detectedPatternSize, cornerSet);
        }
        
        // Apply artwork control (Extension 2)
        if (artworkControl) {
            extractCornersFromImage(detectedCorner, frame, detectedPatternSize, cornerSet);
            calculate_camposition(pointSet, cornerSet, camera_matrix_read, dis_coef_read, rotate_vec, tran_vec, detectedPatternSize);
            if (!cornerSet.empty()) {
                Mat artworkImage;
                applyDesignToChessboard(detectedCorner, artworkImage, cornerSet, camera_matrix_read, dis_coef_read, rotate_vec, tran_vec, detectedPatternSize);
                artworkImage.copyTo(detectedCorner);
                artworkImage.copyTo(displayImage);
            }
        }

        // Task 2
        if (key == 's') 
        {
            extractCornersFromImage(detectedCorner, frame, detectedPatternSize, cornerSet);
            // Task 2 - select calibration images
            selectCalibration(cornerSet, corners_list, pointSet, points_list, detectedPatternSize);

            printf("Saving calibration image...\n");
            std::string fname = "calibration-frame-";
            fname += std::to_string(frameno) + ".jpg";
            imwrite(fname, detectedCorner);
            frameno++;
            key = '1';
        }

        // Task 3
        if (key == 'c') 
        {
            if (corners_list.size() < 5) 
            {
                cout << "Not enough calibration images input, at least 5 images needed. Current # of images: " << corners_list.size() << endl;
            }
            else
            {
                // save current calibration in a csv file
                std::cout << std::endl << "Saving performed calibration..." << std::endl;
                camera_matrix = cv::Mat::eye(3, 3, CV_64FC1);
                camera_matrix.at<double>(0, 2) = frame.cols / 2;
                camera_matrix.at<double>(1, 2) = frame.rows / 2;
                try {
                    //reproj_err = cv::calibrateCamera(points_list, corners_list, displayImage.size(), camera_matrix, dist_coeff, rotate_vec, tran_vec, cv::CALIB_FIX_ASPECT_RATIO, cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 50, 0.1)/*, cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 30, DBL_EPSILON)*/);
                    calibrateCameraHelper(frame, points_list, corners_list, camera_matrix, dist_coeff, rotate_vec, tran_vec, reproj_err);
                }
                catch (Exception e) {
                    cerr << e.what() << endl;
                 }
                cout << endl << "Camera matrix read:" << endl;
                printMatrix(camera_matrix);
                cout << "Distortion coefficient read: " << endl;
                printMatrix(dist_coeff);
                cout << endl << "Rotation matrix: " << endl;
                printMatrix(rotate_vec);
                cout << "Translation matrix: " << endl;
                printMatrix(tran_vec);
                // TODO : calibration images, you might want to also save these rotations and translations with them
                saveCalibration(camera_matrix, dist_coeff);
                std::cout << "re-projection error: " << reproj_err << std::endl;
            }
            key = '1';
        }

        // Task 4
        if (key == 'p')
        {
            extractCornersFromImage(detectedCorner, frame, detectedPatternSize, cornerSet);
            calculate_camposition(pointSet, cornerSet, camera_matrix_read, dis_coef_read, rotate_vec, tran_vec, detectedPatternSize);
            cout << endl << "Camera matrix read:" << endl;
            printMatrix(camera_matrix_read);
            cout << "Distortion coefficient read: " << endl;
            printMatrix(dis_coef_read);
            cout << endl << "Rotation matrix: " << endl;
            printMatrix(rotate_vec);
            cout << "Translation matrix: " << endl;
            printMatrix(tran_vec);
            key = '1';
        }

        // Task 5
        if (key == 'a')
        {
            if (!artworkControl) {
                extractCornersFromImage(detectedCorner, frame, detectedPatternSize, cornerSet);
                calculate_camposition(pointSet, cornerSet, camera_matrix_read, dis_coef_read, rotate_vec, tran_vec, detectedPatternSize);
            }
            cv::Mat axes_image;
            draw3dAxes(detectedCorner, axes_image, camera_matrix_read, dis_coef_read, rotate_vec, tran_vec);
            axes_image.copyTo(displayImage);
        }

        // Task 6 & Extension 3
        if (key == 'v')
        {
            if (!artworkControl) {
                extractCornersFromImage(detectedCorner, frame, detectedPatternSize, cornerSet);
                calculate_camposition(pointSet, cornerSet, camera_matrix_read, dis_coef_read, rotate_vec, tran_vec, detectedPatternSize);
            }
            cv::Mat projected_img;
            draw3dObject(detectedCorner, projected_img, camera_matrix_read, dis_coef_read, rotate_vec, tran_vec);
            draw3dPyramid(projected_img, projected_img, camera_matrix_read, dis_coef_read, rotate_vec, tran_vec);
            projected_img.copyTo(displayImage);
        }

        // Task 7
        if (key == '7')
        {
            findHarrisCorners(frame, displayImage);
        }

        // Video capturing logic
        if (videoWriter.isOpened()) {
            videoWriter.write(displayImage);
        }

        cv::imshow(windowName, displayImage);
        temp = key;
        key = cv::waitKey(10);

        // Implements toggle functionality
        if (key == temp) {
            key = 0;
        }

        if (key == -1) {
            key = temp;
        }

        // Toggle artwork (Extension 2)
        if (key == 't') {
            if (artworkControl) {
                artworkControl = 0;
            }
            else {
                artworkControl = 1;
            }
            key = temp;
        }

        // Start video capture
        if (key == 'b') {
            key = temp;
            if (!videoWriter.isOpened()) {
                std::string fileLocation = "Capture_" + std::to_string(saveCount);
                fileLocation = fileLocation + ".mp4";
                bool isColor = key != '1' && key != '2';
                videoWriter = cv::VideoWriter(fileLocation, cv::VideoWriter::fourcc('M', 'P', '4', 'V'), 30, Size(refS.width, refS.height), isColor);
                // videoWriter.open()
                saveCount++;
                std::cout << "Start capturing video: " << fileLocation << std::endl;
            }
            else {
                std::cout << "Ending video capture" << std::endl;
                videoWriter.release();
            }
        }

        // Save image
        if (key == 'n') {
            key = temp;
            std::string fileLocation = "Capture_" + std::to_string(saveCount);
            fileLocation = fileLocation + ".jpg";
            bool saved = cv::imwrite(fileLocation, displayImage);
            if (saved) {
                cv::namedWindow("Preview Saved File", 1);
                Mat image = imread(fileLocation,
                    IMREAD_UNCHANGED);

                // Error handling
                if (image.empty()) {
                    std::cout << "Image File Not Found" << std::endl;
                    cv::destroyWindow("Save File");
                }
                cv::imshow("Preview Saved File", displayImage);
                saveCount++;
                cv::waitKey(0);
                cv::destroyWindow("Preview Saved File");
            }
        }

        if (key == 'q')
        {
            break;
        }
        // Avoid removal of filter by other key inputs
        if (key != 0 && key != '1' && key != 'a' && key != 'v' && key != 'p' && key != 's' && key != 'c' && key != '7' && key != 't' ) {
            key = temp;
        }
    }

    delete capdev;
    return(0);
}