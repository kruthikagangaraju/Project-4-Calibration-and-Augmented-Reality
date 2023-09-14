/*
Malhar Mahant & Kruthika Gangaraju

CS 5330 Computer Vision
Spring 2023

Defines CPP functions for tasks in Calibration and Augmented Reality.

*/
#ifndef CALIBRATIONFUNCTION_HPP
#define CALIBRATIONFUNCTION_HPP

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

using namespace std;
using namespace cv;

/*
* This function detects the corners of the chessboard.
* Mat frame: Original camera feed
* Mat output: Output frame after corners are detected
* Size patternSize: Number of corners on chessboard
* vector<Point2f> corners: Array of detected corners
*/
void extract_corners(Mat& frame, Mat& output, Size& patternSize, vector<Point2f>& corners);

/*
This function reads the corner points of the chessboard in image frame, creates a list of these points for each calibration image, converts into world frame points and saves those points in a list
vector<Point2f>& corners: Array of corner points
vector<vector<Point2f>>& corners_list: List of all corner points for that image frame
vector<Vec3f>& points: Array of corner points in world frame
vector<vector<Vec3f>>& points_list: List of corner points converted to world frame for that particular image frame
cv::Size boardSize: The size of the identified pattern.
*/
int selectCalibration(vector<Point2f>& corners, vector<vector<Point2f>>& corners_list, vector<Vec3f>& points, vector<vector<Vec3f>>& points_list, cv::Size boardSize);

/*
This function calibrates the camera with the collected corners and points list. It returns the reprojection error.
*/
float calibrateCameraHelper(Mat& frame, vector<vector<Vec3f>>& points_list, vector<vector<Point2f>>& corners_list, Mat& camera_matrix, Mat& dist_coeff, Mat& rotate_vec, Mat& trans_vec, float& error);

/*
This function saves the camera matrix and distortion coeffecient for the saved calibration images to a file.
*/
int saveCalibration(cv::Mat& camera_matrix, cv::Mat& dist_coeff);

/*
* This is a helper method to find the rotational and translational vectors for a given frame using the identified corner points and the saved calibration parameters.
*/
void calculate_camposition(vector<Vec3f>& point_set, vector<Point2f> corner_set, Mat& camera_matrix_read, Mat& dis_coef_read, Mat& rotate_vec, Mat& trans_vec, cv::Size boardSize);

/*
* This is a helper method to draw the 3D axes onto the 2D image using the identified corner points and the saved calibration parameters and calculated rotational and translational vectors for a given frame.
*/
int draw3dAxes(Mat& frame, Mat& axes_img, Mat& camera_matrix, Mat& dist_coeff, Mat& rotate_vec, Mat& trans_vec);

/*
* This is a helper method to draw the 3D cuboid onto the 2D image using the identified corner points and the saved calibration parameters and calculated rotational and translational vectors for a given frame.
*/
int draw3dObject(Mat& frame, Mat& projected_img, Mat& camera_matrix, Mat& dist_coeff, Mat& rotate_vec, Mat& trans_vec);

/*
* This is a helper method to draw the 3D pyramid onto the 2D image using the identified corner points and the saved calibration parameters and calculated rotational and translational vectors for a given frame.
*/
int draw3dPyramid(Mat& frame, Mat& projected_img, Mat& camera_matrix, Mat& dist_coeff, Mat& rotate_vec, Mat& trans_vec);

/*
* This is a helper method to overlay an artwork onto the identified pattern using the identified corner points and the saved calibration parameters and calculated rotational and translational vectors for a given frame.
*/
int applyDesignToChessboard(Mat& frame, Mat& output, vector<Point2f> corners, Mat& camera_matrix, Mat& dist_coeff, Mat& rotate_vec, Mat& trans_vec, cv::Size boardSize);

/*
* This is a helper method to find the Harris Corners in the given frame and return an output with the harris corners drawn on the input frame.
*/
int findHarrisCorners(Mat& frame, Mat& output);
#endif