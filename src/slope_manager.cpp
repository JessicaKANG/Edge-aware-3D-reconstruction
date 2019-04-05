// Copyright (c) 2012, 2013, 2014 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

// Modified by Jessica Kang 2018 17 Oct

#include "stdafx.h"
#include <opencv\cv.h>
#include <opencv2\highgui\highgui.hpp>
#include <iostream>
 
using  namespace cv;
 
int _tmain(int argc, _TCHAR* argv[])
{
	// 读取图像
	Mat img = imread("/home/jessica/jessica/lena.jpg");
 
	// 转换为灰度图像
	Mat gray;
	cvtColor(img,gray,CV_BGR2GRAY);
 
	// 求得x和y方向的一阶微分
	Mat sobelx;
	Mat sobely;
	Sobel(gray, sobelx, CV_32F, 1, 0, 3);
	Sobel(gray, sobely, CV_32F, 0, 1, 3);
 
	// 求得梯度和方向
	Mat norm;
	Mat dir;
	cartToPolar(sobelx, sobely, norm, dir);
 
	// 转换为8位单通道图像进行显示
	double normMax;
	minMaxLoc(norm, NULL, &normMax);
	Mat grad;
	norm.convertTo(grad, CV_8UC1, 255.0/normMax, 0);
 
	double dirMax;
	minMaxLoc(dir, NULL, &dirMax);
	Mat angle;
	dir.convertTo(angle, CV_8UC1, 255.0/dirMax, 0);
	
	namedWindow("grad");
	imshow("grad",grad);
	namedWindow("angle");
	imshow("angle",angle);
 
	waitKey(0);
	img.release();
	gray.release();
	sobelx.release();
	sobely.release();
	norm.release();
	dir.release();
	grad.release();
	angle.release();
 
	return 0;
}