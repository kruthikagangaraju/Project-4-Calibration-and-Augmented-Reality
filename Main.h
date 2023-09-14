/*
Malhar Mahant & Kruthika Gangaraju

CS 5330 Computer Vision
Spring 2023

Defines CPP helper functions used in Main file.

*/
#pragma once

/*
* Helper method to extract corners of a chessboard or asymmetric circles grid pattern from the image.
*/
void extractCornersFromImage(cv::Mat& detectedCorner, cv::Mat& frame, cv::Size& patternSize, std::vector<cv::Point2f>& cornerSet);
