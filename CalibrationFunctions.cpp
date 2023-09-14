/*
Malhar Mahant & Kruthika Gangaraju

CS 5330 Computer Vision
Spring 2023

Implements CPP functions for tasks in Calibration and Augmented Reality.

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

string csv_name = "intrinsic_data.csv";
char* csv_file = &csv_name[0];
string csv_name_circle_grid = "intrinsic_data_circle_grid.csv";
char* csv_file_circle_grid = &csv_name_circle_grid[0];
vector<char*> cam_mat;
vector<vector<float>> parameters;
bool calibrationUpdated = false;
bool patternUpdated = false;
bool patternIsChessboard = true;
bool oldValue = false;

/*
* Helper method to print the given matrix to the console
*/
void printMatrixToConsole(cv::Mat& matrix) {
    for (int i = 0; i < matrix.rows; i++) {
        for (int j = 0; j < matrix.cols; j++) {
            cout << matrix.at<double>(i, j) << "\t";
        }
        cout << endl;
    }
}


/*
* This function detects the corners of the chessboard.
* Mat frame: Original camera feed
* Mat output: Output frame after corners are detected
* Size patternSize: Number of corners on chessboard
* vector<Point2f> corners: Array of detected corners
*/
void extract_corners(cv::Mat& frame, cv::Mat& output, cv::Size& patternSize, std::vector<cv::Point2f>& corners)
{
    corners.clear();
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    patternSize.width = 9;
    patternSize.height = 6;
    bool pattern_found = cv::findChessboardCorners(frame, patternSize, corners);
    if (pattern_found) {
        patternIsChessboard = pattern_found;
        cv::cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 50, 0.1));
    }
    else {
        corners.clear();
        patternSize.width = 4;
        patternSize.height = 11;
        try {
            pattern_found = cv::findCirclesGrid(frame, patternSize, corners, cv::CALIB_CB_ASYMMETRIC_GRID);
        }
        catch (Exception e) {
            //std::cerr << e.what();
        }
        if (pattern_found) {
            patternIsChessboard = !pattern_found;
        }
    }
    frame.copyTo(output);
    patternUpdated = patternIsChessboard != oldValue;
    oldValue = patternIsChessboard;
    cv::drawChessboardCorners(output, patternSize, Mat(corners), pattern_found);
}

/*
This function reads the corner points of the chessboard in image frame, creates a list of these points for each calibration image, converts into world frame points and saves those points in a list
vector<Point2f>& corners: Array of corner points
vector<vector<Point2f>>& corners_list: List of all corner points for that image frame
vector<Vec3f>& points: Array of corner points in world frame
vector<vector<Vec3f>>& points_list: List of corner points converted to world frame for that particular image frame
cv::Size boardSize: The size of the identified pattern.
*/

int selectCalibration(vector<Point2f>& corners, vector<vector<Point2f>>& corners_list, vector<Vec3f>& points, vector<vector<Vec3f>>& points_list, cv::Size boardSize)
{
    int point_list_size = points_list.size();
    int corner_list_size = corners_list.size();

    points.clear();
    for (int i = 0; i > -boardSize.height; i--) {
        for (int j = 0; j < boardSize.width; j++) {
            points.push_back(Vec3f(j, i, 0));
        }
    }

    corners_list.push_back(corners);
    points_list.push_back(points);
    cout << "points size " << points.size() << endl;
    cout << "corners size " << corners.size() << endl;
    return(0);
}

/*
This function calibrates the camera with the collected corners and points list. It stores the reprojection error and displays it.
*/
float calibrateCameraHelper(Mat& frame, vector<vector<Vec3f>>& points_list, vector<vector<Point2f>>& corners_list, Mat& camera_matrix, Mat& dist_coeff, Mat& rotate_vec, Mat& trans_vec, float& error)
{
    try
    {
        /*error = cv::calibrateCamera(points_list,
            corners_list,
            frame.size(),
            camera_matrix,
            dist_coeff,
            rot,
            trans,
            cv::CALIB_FIX_ASPECT_RATIO,
            cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 30, DBL_EPSILON));*/
        if (patternIsChessboard) {
            error = cv::calibrateCamera(points_list, corners_list, frame.size(), camera_matrix, dist_coeff, rotate_vec, trans_vec, cv::CALIB_FIX_ASPECT_RATIO, /*,cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 50, 0.1) */ cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 30, DBL_EPSILON));
        }
        else {
            error = cv::calibrateCamera(points_list, corners_list, frame.size(), camera_matrix, dist_coeff, rotate_vec, trans_vec, cv::CALIB_FIX_ASPECT_RATIO);
        }
    }
    catch (const std::exception& exc)
    {
        // catch anything thrown within try block that derives from std::exception
        //std::cerr << exc.what();
    }

    return 0;
}

/*
This function saves the camera matrix and distortion coeffecient for the saved calibration images to a file.
*/
int saveCalibration(cv::Mat& camera_matrix, cv::Mat& dist_coeff)
{
    std::string columnName = "camera_matrix";
    char* labelName_char = new char[columnName.length() + 1];
    strcpy(labelName_char, columnName.c_str());

    std::vector<float> camVector;
    for (int i = 0; i < camera_matrix.rows; i++) {
        for (int j = 0; j < camera_matrix.cols; j++) {
            float f_val = (float)camera_matrix.at<double>(i, j);
            camVector.push_back(f_val);
        }
    }
    if (patternIsChessboard) {
        append_image_data_csv(csv_file, labelName_char, camVector, 1);
    }
    else {
        append_image_data_csv(csv_file_circle_grid, labelName_char, camVector, 1);
    }
    columnName = "distortion_coeff";
    char* label_char = new char[columnName.length() + 1];
    strcpy(label_char, columnName.c_str());

    std::vector<float> distVector;
    for (int i = 0; i < dist_coeff.rows; i++) {
        for (int j = 0; j < dist_coeff.cols; j++) {
            float f_val = (float)dist_coeff.at<double>(i, j);
            distVector.push_back(f_val);
        }
    }
    if (patternIsChessboard) {
        append_image_data_csv(csv_file, label_char, distVector);
    }
    else {
        append_image_data_csv(csv_file_circle_grid, label_char, distVector);
    }
    calibrationUpdated = true;
    cam_mat.clear();
    parameters.clear();
    return 0;
}

/*
* This is a helper method to find the rotational and translational vectors for a given frame using the identified corner points and the saved calibration parameters.
*/
void calculate_camposition(vector<Vec3f>& point_set, vector<Point2f> corner_set,
    Mat& camera_matrix_read, Mat& dis_coef_read, Mat& rotate_vec, Mat& trans_vec, cv::Size boardSize) {
    // read camera matrix and distortion coefficient from csv file
    double camera_matrix_2Darray_read[3][3];
    double dis_coef_2Darray_read[1][5];
    if (cam_mat.empty() || parameters.empty() || calibrationUpdated || patternUpdated) {
        if (patternIsChessboard) {
            read_image_data_csv(csv_file, cam_mat, parameters);
        }
        else {
            read_image_data_csv(csv_file_circle_grid, cam_mat, parameters);
        }
        calibrationUpdated = false;
    }
    int c = 0;
    while (c < parameters[0].size())
    {
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                camera_matrix_2Darray_read[i][j] = parameters[0][c];
                cout << parameters[0][c] << endl;
                c++;
            }
        }
    }
    c = 0;
    while (c < parameters[1].size()) {
        for (int i = 0; i < 1; i++) {
            for (int j = 0; j < 5; j++) {
                dis_coef_2Darray_read[i][j] = parameters[1][c];
                cout << parameters[1][c] << endl;
                c++;
            }
        }
    }
    camera_matrix_read = cv::Mat(3, 3, CV_64FC1, &camera_matrix_2Darray_read);
    dis_coef_read = cv::Mat(1, 5, CV_64FC1, &dis_coef_2Darray_read);
    point_set.clear();
    for (int i = 0; i > -boardSize.height; i--) {
        for (int j = 0; j < boardSize.width; j++) {
            point_set.push_back(cv::Vec3f(j, i, 0));
        }
    }
    try
    {
        cv::solvePnP(point_set, corner_set, camera_matrix_read, dis_coef_read, rotate_vec, trans_vec);
    }
    catch (Exception e)
    {
        // catch anything thrown within try block that derives from std::exception
        //std::cerr << e.msg;
    }

    camera_matrix_read = camera_matrix_read.clone();
    dis_coef_read = dis_coef_read.clone();
}

/*
* This is a helper method to draw the 3D axes onto the 2D image using the identified corner points and the saved calibration parameters and calculated rotational and translational vectors for a given frame.
*/
int draw3dAxes(Mat& frame, Mat& axes_img, Mat& camera_matrix, Mat& dist_coeff, Mat& rotate_vec, Mat& trans_vec)
{
    std::vector<cv::Vec3f> points;
    frame.copyTo(axes_img);
    points.push_back(Vec3f({ 0, 0, 0 }));
    points.push_back(Vec3f({ 2, 0, 0 }));
    points.push_back(Vec3f({ 0, -2, 0 }));
    points.push_back(Vec3f({ 0, 0, 2 }));

    std::vector<cv::Point2f> corners;
    try {
        cv::projectPoints(points, rotate_vec, trans_vec, camera_matrix, dist_coeff, corners);
    }
    catch (Exception e)
    {
        // catch anything thrown within try block that derives from std::exception
        //std::cerr << e.msg;
    }
    cv::arrowedLine(axes_img, corners[0], corners[1], cv::Scalar(0, 0, 255), 2);
    cv::arrowedLine(axes_img, corners[0], corners[2], cv::Scalar(0, 255, 0), 2);
    cv::arrowedLine(axes_img, corners[0], corners[3], cv::Scalar(255, 0, 0), 2);
    return 0;
}

/*
* This is a helper method to draw the 3D cuboid onto the 2D image using the identified corner points and the saved calibration parameters and calculated rotational and translational vectors for a given frame.
*/
int draw3dObject(Mat& frame, Mat& projected_img, Mat& camera_matrix, Mat& dist_coeff, Mat& rotate_vec, Mat& trans_vec)
{
    std::vector<cv::Vec3f> vertex;
    frame.copyTo(projected_img);
    std::vector<cv::Point2f> object;

    // CUBOID
    vertex.push_back(cv::Vec3f({ 0, 0, 0 })); // br
    vertex.push_back(cv::Vec3f({ 0, -2, 0 }));
    vertex.push_back(cv::Vec3f({ 3, 0, 0 })); // bl
    vertex.push_back(cv::Vec3f({ 3, -2, 0 }));
    vertex.push_back(cv::Vec3f({ 0, 0, 5 })); // tr
    vertex.push_back(cv::Vec3f({ 0, -2, 5 }));
    vertex.push_back(cv::Vec3f({ 3, 0, 5 })); // tl
    vertex.push_back(cv::Vec3f({ 3, -2, 5 }));

    cv::projectPoints(vertex, rotate_vec, trans_vec, camera_matrix, dist_coeff, object);

    for (int i = 0; i < object.size(); i++)
    {   
        for (int j = 0; j < object.size(); j++)
        {
            cv::line(projected_img, object[i], object[j], Scalar(255, 255, 0), 2);
        }
    }

    vertex.clear();
    object.clear();

    return 0;
}

/*
* This is a helper method to draw the 3D pyramid onto the 2D image using the identified corner points and the saved calibration parameters and calculated rotational and translational vectors for a given frame.
*/
int draw3dPyramid(Mat& frame, Mat& projected_img, Mat& camera_matrix, Mat& dist_coeff, Mat& rotate_vec, Mat& trans_vec)
{
    std::vector<cv::Vec3f> vertex;
    frame.copyTo(projected_img);
    std::vector<cv::Point2f> object;

    // PYRAMID
    vertex.push_back(cv::Vec3f({ 7, -1, 3 }));
    vertex.push_back(cv::Vec3f({ 6, 0, 0 }));
    vertex.push_back(cv::Vec3f({ 8, 0, 0 }));
    vertex.push_back(cv::Vec3f({ 6, -2, 0 }));
    vertex.push_back(cv::Vec3f({ 8, -2, 0 }));

    cv::projectPoints(vertex, rotate_vec, trans_vec, camera_matrix, dist_coeff, object);

    for (int i = 0; i < object.size(); i++)
    {
        for (int j = 0; j < object.size(); j++)
        {
            cv::line(projected_img, object[i], object[j], Scalar(255, 155, 0), 2);
        }
    }

    vertex.clear();
    object.clear();

    return 0;
}

/*
* This is a helper method to overlay an artwork onto the identified pattern using the identified corner points and the saved calibration parameters and calculated rotational and translational vectors for a given frame.
*/
int applyDesignToChessboard(Mat& frame, Mat& output, vector<Point2f> corners, Mat& camera_matrix, Mat& dist_coeff, Mat& rotate_vec, Mat& trans_vec, cv::Size boardSize) {
    cv::Mat artwork = cv::imread("artwork.jpg");
  
    std::vector<cv::Point2f> overlayCoords = { cv::Point2f(0, 0),
                                               cv::Point2f(0, artwork.rows),
                                               cv::Point2f(artwork.cols, artwork.rows),
                                               cv::Point2f(artwork.cols, 0) };
   
    std::vector<cv::Point2f> src;

    std::vector<cv::Vec3f> vertex;
    vertex.push_back(cv::Vec3f({ -1, 1, 0 }));
    vertex.push_back(cv::Vec3f({ -1, - float(boardSize.height), 0 }));
    vertex.push_back(cv::Vec3f({ float(boardSize.width), -float(boardSize.height), 0 }));
    vertex.push_back(cv::Vec3f({ float(boardSize.width), 1, 0 }));
    try {
        cv::projectPoints(vertex, rotate_vec, trans_vec, camera_matrix, dist_coeff, src);
    }
    catch (Exception e) {
        //cerr << e.what() << endl;
    }
    cv::Mat H;
   
    H = cv::getPerspectiveTransform(overlayCoords, src);
   
    cv::Mat warpedOverlay;
    cv::warpPerspective(artwork, warpedOverlay, H, frame.size());

    cv::Mat mask(frame.size(), CV_8UC1, cv::Scalar(0));
    std::vector<cv::Point2i> srcIntegers(src.begin(), src.end());
    try {
        //cout << srcIntegers.size() << endl;
        //cout << srcIntegers[0] << endl;
        //cout << srcIntegers[1] << endl;
        //cout << srcIntegers[2] << endl;
        //cout << srcIntegers[3] << endl;
        cv::fillConvexPoly(mask, srcIntegers, cv::Scalar(255));
    }
    catch (Exception e) {
        //cerr << e.what() << endl;
    }
    cv::Mat invertedMask;
    cv::bitwise_not(mask, invertedMask);

    cv::Mat outside;
    cv::bitwise_and(frame, frame, outside, invertedMask);
    try {
        cv::add(outside, warpedOverlay, output);
    }
    catch (Exception e) {
        //cerr << " H#Y " << e.what() << endl;
    }
    return 0;
}

/*
* This is a helper method to find the Harris Corners in the given frame and return an output with the harris corners drawn on the input frame.
*/
int findHarrisCorners(Mat& frame, Mat& output) {
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    Mat harrisCorners;
    cornerHarris(gray, harrisCorners, 3, 3, 0.04);
    normalize(harrisCorners, harrisCorners, 0, 255, NORM_MINMAX);
    convertScaleAbs(harrisCorners, harrisCorners);
    output = frame.clone();
    for (int i = 0; i < harrisCorners.rows; i++) {
        for (int j = 0; j < harrisCorners.cols; j++) {
            if ((int)harrisCorners.at<uchar>(i, j) > 120) {
                circle(output, Point(j, i), 5, Scalar(0, 0, 255), 2, 8, 0);
            }
        }
    }
    return 0;
}