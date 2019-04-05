// Copyright (c) 2012, 2013, 2014 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

// Modified by Jessica Kang 2018 17 Oct

#include <cstdlib>

//#include "openMVG/sfm/pipelines/global/sfm_global_engine_relative_motions.hpp"
#include "G2OEngine.hpp"
#include "openMVG/system/timer.hpp"
#include "openMVG/cameras/Cameras_Common_command_line_helper.hpp"

using namespace openMVG;
using namespace openMVG::sfm;

#include "third_party/cmdLine/cmdLine.h"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"

#include "slope_manager.hpp"

#include "opencv2/imgproc/imgproc.hpp"
#include <opencv/cv.h>
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
 
using  namespace cv;

/* output slope angle to HSV space */
std::map <IndexT, cv::Mat> slope_to_HSV(std::map <std::pair<IndexT, std::pair<double, double>>, double> map_angle, int cols, int rows)
{
  std::map <IndexT, cv::Mat> output;

  for(auto it = map_angle.begin(); it != map_angle.end(); ++it){
    IndexT view_id = it->first.first;
    std::pair<double, double> pixel = it->first.second;
    double angle = it->second;

    if(output.find(view_id) == output.end()){
      output[view_id] = cv::Mat::zeros(rows, cols, CV_8U);
    }
    int x0 = round(pixel.first);
    int y0 = round(pixel.second);
    output[view_id].at<uchar>(y0,x0) = angle;
    output[view_id].at<uchar>(y0-1,x0) = angle;
    output[view_id].at<uchar>(y0,x0-1) = angle;
    output[view_id].at<uchar>(y0+1,x0) = angle;
    output[view_id].at<uchar>(y0,x0+1) = angle;
  }
  return output;
}
void write_HSV(std::map <IndexT, cv::Mat> output, string output_dir, int cols, int rows, int it_count)
{
  for(auto it = output.begin(); it != output.end(); ++it){
    cv::Mat dir = it->second;
    std::vector<cv::Mat> HSV;
    cv::Mat angle = cv::Mat(rows, cols, CV_8U);
    cv::Mat ones = cv::Mat::ones(rows, cols, CV_8U);
    cv::Mat S = ones.mul(dir) * 255;
    cv::Mat V = ones.mul(dir) * 255;

    double dirMax = 6.29;
    //minMaxLoc(dir, NULL, &dirMax);
    cv::Mat H;
    dir.convertTo(H, CV_8UC1, 180/dirMax, 0);
    HSV.push_back(H);
    HSV.push_back(S);
    HSV.push_back(V);
    merge(HSV,angle);
    cv::Mat rgb_angle;
    cvtColor(angle, rgb_angle, CV_HSV2BGR);
    
    char image_dir[20];
    sprintf(image_dir, "/slopes/%04d_%d.png", it->first, it_count);
    string out_fileFullName = output_dir + image_dir;
    imwrite(out_fileFullName, rgb_angle);
  }
}
void write_HSV(cv::Mat dir, string output_dir, int cols, int rows){
    std::vector<cv::Mat> HSV;
    cv::Mat angle = cv::Mat(rows, cols, CV_8U);
    cv::Mat ones = cv::Mat::ones(rows, cols, CV_8U);
    
    double dirMax;
    minMaxLoc(dir, NULL, &dirMax);
    cv::Mat H;
    //std::cout<< dirMax << std::endl;
    dir.convertTo(H, CV_8UC1, 180/dirMax, 0);

    HSV.push_back(H);
    cv::Mat S = ones.mul(H) * 255;
    cv::Mat V = ones.mul(H) * 255;
    HSV.push_back(S);
    HSV.push_back(V);
    merge(HSV,angle);
    cv::Mat rgb_angle;
    cvtColor(angle, rgb_angle, CV_HSV2BGR);
   
    imwrite(output_dir, rgb_angle);
}

void update_map_angle(std::map <IndexT, vector<Vec2>> & map_viewID_pixels, std::map <std::pair<IndexT, std::pair<double, double>>, double> & map_angle,
		      SfM_Data & sfm_data, std::map <IndexT, cv::Mat> map_dir)
{
    map_viewID_pixels.clear();
    map_angle.clear();
    // from sfm_data to map_viewID_pixels
    for (Landmarks::iterator iterTracks = sfm_data.structure.begin();
	iterTracks!= sfm_data.structure.end(); ++iterTracks)
	{
	  const Observations & obs = iterTracks->second.obs;
	  for (Observations::const_iterator itObs = obs.begin();
		itObs != obs.end(); ++itObs)
		{
		  IndexT indexView = itObs->first;
		  Vec2 x = itObs->second.x;
		  map_viewID_pixels[indexView].push_back(x);
		}
	    
	}
    // from map_viewID_pixels to map_angle
    for(auto it = map_viewID_pixels.begin(); it != map_viewID_pixels.end(); ++it){
      IndexT indexView = it->first;
      vector<Vec2> pixels = it->second;  
      // save slope to angle_map
      for(unsigned int i=0; i<pixels.size(); ++i){
	double x1 = pixels[i](0, 0);
	double y1 = pixels[i](1, 0);
	int x0 = round(x1);
	int y0 = round(y1);
	cv::Mat dir = map_dir[indexView];
	float data = dir.at<float>(y0,x0);
	//uchar* data= dir.ptr<uchar>(y0);
	map_angle[std::make_pair(indexView, std::make_pair(x1,y1))] = data;
      }

    }
}

void fix_null_camera(SfM_Data & sfm_data)
{
  for (Poses::iterator itPose = sfm_data.poses.begin(); 
       itPose != sfm_data.poses.end(); ++itPose)
       {

	  Pose3 & pose = itPose->second;
	  //Mat3 R = pose.rotation();
	  Vec3 t = pose.translation();
	  
	  if(t(0,0)==0&&t(1,0)==0&&t(2,0)==0){
	    // null camera
	    std::cout<<"camera to fix: "<< pose.rotation()<< std::endl;	    
	    Mat3 R_refined = pose.rotation()- pose.rotation();
	    Vec3 t_refined = pose.translation();
	    // Update the pose
	    pose = Pose3(R_refined, -R_refined.transpose() * t_refined); 
	    std::cout<<"camera fixed: "<< pose.rotation()<< std::endl;
	  }
	  
	}  
  
}

int main(int argc, char **argv)
{
  using namespace std;
  std::cout << std::endl
    << "-----------------------------------------------------------\n"
    << "OpenEDGE G2O BA Process:\n"
    << "-----------------------------------------------------------\n"
    << "Created by Jessica Kang 2018 17 Oct"<< std::endl
    << "------------------------------------------------------------"
    << std::endl;


      CmdLine cmd;

      std::string sSfM_Data_Filename;
      //* std::string sMatchesDir;
      std::string sOutDir = "";
      int iRotationAveragingMethod = int (ROTATION_AVERAGING_L2);
      int iTranslationAveragingMethod = int (TRANSLATION_AVERAGING_SOFTL1);
      std::string sIntrinsic_refinement_options = "ADJUST_ALL";
      
      bool sobel_flag = false;
      bool g2o_edge_flag = true;
      double m = 1;

      cmd.add( make_option('i', sSfM_Data_Filename, "input_file") );
      //* cmd.add( make_option('m', sMatchesDir, "matchdir") );
      cmd.add( make_option('o', sOutDir, "outdir") );
      cmd.add( make_option('r', iRotationAveragingMethod, "rotationAveraging") );
      cmd.add( make_option('t', iTranslationAveragingMethod, "translationAveraging") );
      cmd.add( make_option('f', sIntrinsic_refinement_options, "refineIntrinsics") );
      cmd.add( make_option('s', sobel_flag, "sobel_flag"));
      cmd.add( make_option('m', m, "edge_ratio"));
      cmd.add( make_option('g', g2o_edge_flag, "g2o_edge_flag"));

      try {
	if (argc == 1) throw std::string("Invalid parameter.");
	cmd.process(argc, argv);
      } catch(const std::string& s) {
	std::cerr << "Usage: " << argv[0] << '\n'
	<< "[-i|--input_file] path to a SfM_Data scene\n"
	<< "[-o|--outdir] path where the output data will be stored\n"
	<< "\n[Optional]\n"
	<< "[-r|--rotationAveraging]\n"
	  << "\t 1 -> L1 minimization\n"
	  << "\t 2 -> L2 minimization (default)\n"
	<< "[-t|--translationAveraging]:\n"
	  << "\t 1 -> L1 minimization\n"
	  << "\t 2 -> L2 minimization of sum of squared Chordal distances\n"
	  << "\t 3 -> SoftL1 minimization (default)\n"
	<< "[-f|--refineIntrinsics] Intrinsic parameters refinement option\n"
	  << "\t ADJUST_ALL -> refine all existing parameters (default) \n"
	  << "\t NONE -> intrinsic parameters are held as constant\n"
	  << "\t ADJUST_FOCAL_LENGTH -> refine only the focal length\n"
	  << "\t ADJUST_PRINCIPAL_POINT -> refine only the principal point position\n"
	  << "\t ADJUST_DISTORTION -> refine only the distortion coefficient(s) (if any)\n"
	  << "\t -> NOTE: options can be combined thanks to '|'\n"
	  << "\t ADJUST_FOCAL_LENGTH|ADJUST_PRINCIPAL_POINT\n"
	  <<      "\t\t-> refine the focal length & the principal point position\n"
	  << "\t ADJUST_FOCAL_LENGTH|ADJUST_DISTORTION\n"
	  <<      "\t\t-> refine the focal length & the distortion coefficient(s) (if any)\n"
	  << "\t ADJUST_PRINCIPAL_POINT|ADJUST_DISTORTION\n"
	  <<      "\t\t-> refine the principal point position & the distortion coefficient(s) (if any)\n"
	<< std::endl;

	std::cerr << s << std::endl;
	return EXIT_FAILURE;
      }
    
  if (iRotationAveragingMethod < ROTATION_AVERAGING_L1 ||
      iRotationAveragingMethod > ROTATION_AVERAGING_L2 )  {
    std::cerr << "\n Rotation averaging method is invalid" << std::endl;
    return EXIT_FAILURE;
  }

  const cameras::Intrinsic_Parameter_Type intrinsic_refinement_options =
    cameras::StringTo_Intrinsic_Parameter_Type(sIntrinsic_refinement_options);

  if (iTranslationAveragingMethod < TRANSLATION_AVERAGING_L1 ||
      iTranslationAveragingMethod > TRANSLATION_AVERAGING_SOFTL1 )  {
    std::cerr << "\n Translation averaging method is invalid" << std::endl;
    return EXIT_FAILURE;
  }

  // Load input SfM_Data scene
  SfM_Data sfm_data;
  if (!Load(sfm_data, sSfM_Data_Filename, ESfM_Data(ALL))) {
    std::cerr << std::endl
      << "The input SfM_Data file \""<< sSfM_Data_Filename << "\" cannot be read." << std::endl;
    return EXIT_FAILURE;
  }

  if (sOutDir.empty())  {
    std::cerr << "\nIt is an invalid output directory" << std::endl;
    return EXIT_FAILURE;
  }

  if (!stlplus::folder_exists(sOutDir))
  {
    if (!stlplus::folder_create(sOutDir))
    {
      std::cerr << "\nCannot create the output directory" << std::endl;
    }
  }
  
  //*************SLOPE MANAGER START HERE***************************************************
  
  int cols, rows;
  std::cout << "Slope manager starts working..." << std::endl;
  
  // construct my data structure
  std::map <IndexT, vector<Vec2>> map_viewID_pixels;
  std::map <std::pair<IndexT, std::pair<double, double>>, double> map_angle;

  // from sfm_data to map_viewID_pixels
  for (Landmarks::iterator iterTracks = sfm_data.structure.begin();
       iterTracks!= sfm_data.structure.end(); ++iterTracks)
       {
	 const Observations & obs = iterTracks->second.obs;
	 for (Observations::const_iterator itObs = obs.begin();
	      itObs != obs.end(); ++itObs)
	      {
		IndexT indexView = itObs->first;
		Vec2 x = itObs->second.x;
		map_viewID_pixels[indexView].push_back(x);
	      }
	   
       }
  // from map_viewID_pixels to map_angle
  std::map <IndexT, cv::Mat> map_dir; //存储所有view的梯度角度
  if(sobel_flag){
    for(auto it = map_viewID_pixels.begin(); it != map_viewID_pixels.end(); ++it){
      IndexT indexView = it->first;
      vector<Vec2> pixels = it->second;
      
      // load image and compute slope with sobel
      char image_dir[20];
      sprintf(image_dir, "/images/%04d.png", indexView);
      std::cout<< sOutDir + image_dir<< std::endl;
      // 读取图像
      cv::Mat img = imread(sOutDir + image_dir);
      cols = img.cols;
      rows = img.rows;
      /////////////////////////////////////
      // 转换为灰度图像
      cv::Mat gray;
      cvtColor(img,gray,CV_BGR2GRAY);
      ////////////////////////////////////

      // 求得x和y方向的一阶微分
      cv::Mat sobelx;
      cv::Mat sobely;
      Sobel(gray, sobelx, CV_32F, 1, 0, 3);
      Sobel(gray, sobely, CV_32F, 0, 1, 3);
      

      // 求得梯度和方向
      cv::Mat norm;
      cv::Mat dir;
      double dirMax;
      cartToPolar(sobely, sobelx, norm, dir);
      
      map_dir[indexView] = dir;
      minMaxLoc(dir, NULL, &dirMax);
      std::cout<<dirMax<<std::endl;
      // save grident image
      char temp_dir[20];
      sprintf(temp_dir, "/gridents/%04d.png", indexView);
      string out_fileFullName = sOutDir + temp_dir;
      write_HSV(dir, out_fileFullName, cols, rows);
      
      // save slope to angle_map
      for(unsigned int i=0; i<pixels.size(); ++i){
	double x1 = pixels[i](0, 0);
	double y1 = pixels[i](1, 0);
	int x0 = round(x1);
	int y0 = round(y1);
	float data = dir.at<float>(y0,x0);
	//uchar* data= dir.ptr<uchar>(y0);
	map_angle[std::make_pair(indexView, std::make_pair(x1,y1))] = data;
      }

    }
  }else{
    for(auto it = map_viewID_pixels.begin(); it != map_viewID_pixels.end(); ++it){
      IndexT indexView = it->first;
      vector<Vec2> pixels = it->second;
      double counter = 0;
      
      // load image to get cols and rows
      char image_dir[20];
      sprintf(image_dir, "/images/%04d.png", indexView);
      std::cout<< sOutDir + image_dir<< std::endl;
      // 读取图像
      cv::Mat img = imread(sOutDir + image_dir);
      cols = img.cols;
      rows = img.rows;

      double min_ang = 7;
      double max_ang = 0;
      // go through all pixels find closest pixel for slope calculation
      for(unsigned int i=0; i< pixels.size(); ++i){
	double x1 = pixels[i](0, 0);
	double y1 = pixels[i](1, 0);
	double min_dist = DBL_MAX;
	double slp;
	int min;
	double dist_threshold = 10;
	for(unsigned int j=0; j< pixels.size(); ++j){
	  if (j!=i){
	  
	    double x2 = pixels[j](0, 0);
	    double y2 = pixels[j](1, 0);
	    if (x1 != x2 || y1 != y2){
	      double dist = distance(x1, y1, x2, y2);
	      if (dist < min_dist){
		min = j;
		min_dist = dist;
		slp = slope(x1, y1, x2, y2);
	      }
	    }
	  }
	}
	// filter slopes with dist_threshold
	if (min_dist < dist_threshold){
	  double a = angle(x1, y1, pixels[min](0,0), pixels[min](1,0));
	  map_angle[std::make_pair(indexView, std::make_pair(x1, y1))] = a;
	  //std::cout<< "angle "<< a << std::endl;
	  if(a<min_ang) min_ang = a;
	  if(a>max_ang) max_ang = a;
	  counter++;
	}
	//std::cout<< "minimal dist " << min_dist << std::endl;
	//std::cout<< "closest point " << pixels[min](0,0) << "," << pixels[min](1,0) << std::endl;
	//std::cout<< "view "<< indexView << " pixel " << pixels[i](0, 0) << "," << pixels[i](1, 0) << " slope is " << slp << std::endl;
      }
      std::cout<<"min_ang: "<<min_ang<<std::endl;
      std::cout<<"max_ang: "<<max_ang<<std::endl;
      std::cout<< "unfiltered percentage "<< counter/ pixels.size() << std::endl;
      std::cout<< "Slope manager finished. "<< std::endl;

    }
  }
  //************************OUTPUT SLOPE INFO*****************************************************************

  std::map <IndexT, cv::Mat> output = slope_to_HSV(map_angle, cols, rows);
  write_HSV(output, sOutDir, cols, rows, 0);

  //************************SLOPE MANAGER DONE****************************************************************

  //---------------------------------------
  // OpenEDGE G2O BA process
  //---------------------------------------

  openMVG::system::Timer timer;
  G2OEngine sfmEngine(
    sfm_data,
    sOutDir,
    stlplus::create_filespec(sOutDir, "Reconstruction_Report.html"));

//   // Configure the features_provider & the matches_provider
//   sfmEngine.SetFeaturesProvider(feats_provider.get());
//   sfmEngine.SetMatchesProvider(matches_provider.get());

  // Configure reconstruction parameters
  sfmEngine.Set_Intrinsics_Refinement_Type(intrinsic_refinement_options);

  // Configure motion averaging method
  sfmEngine.SetRotationAveragingMethod(
    ERotationAveragingMethod(iRotationAveragingMethod));
  sfmEngine.SetTranslationAveragingMethod(
    ETranslationAveragingMethod(iTranslationAveragingMethod));
  
  if(g2o_edge_flag){
    if (sfmEngine.Adjust(map_angle, std::make_pair(m, 1)))
    {
      std::cout << std::endl << " G2O_BA took (s): " << timer.elapsed() << std::endl;
      
      SfM_Data internal_sfm_data = sfmEngine.Get_SfM_Data();
      
      fix_null_camera(internal_sfm_data);

      std::cout << "...Generating SfM_Report.html" << std::endl;
      Generate_SfM_Report(internal_sfm_data,
	stlplus::create_filespec(sOutDir, "SfMReconstruction_Report.html"));

      //-- Export to disk computed scene (data & visualizable results)
      std::cout << "...Export SfM_Data to disk." << std::endl;
      Save(internal_sfm_data,
	stlplus::create_filespec(sOutDir, "sfm_data", ".bin"),
	ESfM_Data(ALL));
      std::string str = std::to_string(m);
      Save(internal_sfm_data,
	stlplus::create_filespec(sOutDir, str, ".json"),
	ESfM_Data(ALL));

      Save(internal_sfm_data,
	stlplus::create_filespec(sOutDir, "cloud_and_poses", ".ply"),
	ESfM_Data(ALL));
      
      return EXIT_SUCCESS;
    }
    return EXIT_FAILURE;
  }
  else{
    if (sfmEngine.Adjust())
    {
      std::cout << std::endl << " G2O_BA took (s): " << timer.elapsed() << std::endl;
      
      SfM_Data internal_sfm_data = sfmEngine.Get_SfM_Data();
      fix_null_camera(internal_sfm_data);

      std::cout << "...Generating SfM_Report.html" << std::endl;
      Generate_SfM_Report(internal_sfm_data,
	stlplus::create_filespec(sOutDir, "SfMReconstruction_Report.html"));

      //-- Export to disk computed scene (data & visualizable results)
      std::cout << "...Export SfM_Data to disk." << std::endl;
      Save(internal_sfm_data,
	stlplus::create_filespec(sOutDir, "sfm_data", ".bin"),
	ESfM_Data(ALL));
      Save(internal_sfm_data,
	stlplus::create_filespec(sOutDir, "sfm_data_org", ".json"),
	ESfM_Data(ALL));

      Save(internal_sfm_data,
	stlplus::create_filespec(sOutDir, "cloud_and_poses", ".ply"),
	ESfM_Data(ALL));
      return EXIT_SUCCESS;
    }
    return EXIT_FAILURE;
  } 
}


/*
 *   if(g2o_edge_flag){
    
    if(sobel_flag){
      std::cout<<"soble edge slope, change angle per iteration"<< std::endl;
      for(int i=1; i<2; i++){
	if (sfmEngine.Adjust(map_angle, std::make_pair(m, 1))){
	  std::cout<<"iteration: "<< i<<" done."<<std::endl;
	  update map_angle
	  update_map_angle(map_viewID_pixels,map_angle, sfm_data, map_dir);  
	  std::map <IndexT, cv::Mat> output = slope_to_HSV(map_angle, cols, rows);
	  write_HSV(output, sOutDir, cols, rows, i);
	}else return EXIT_FAILURE;
      }
       -- Export to disk computed scene (data & visualizable results)
      std::cout << "...Export SfM_Data to disk." << std::endl;
      Save(sfmEngine.Get_SfM_Data(),
	stlplus::create_filespec(sOutDir, "sfm_data", ".bin"),
	ESfM_Data(ALL));
      Save(sfmEngine.Get_SfM_Data(),
	stlplus::create_filespec(sOutDir, "sfm_data", ".json"),
	ESfM_Data(ALL));

      Save(sfmEngine.Get_SfM_Data(),
	stlplus::create_filespec(sOutDir, "cloud_and_poses", ".ply"),
	ESfM_Data(ALL));
      return EXIT_SUCCESS;
    }
    else{
      std::cout<<"nearest neighbour slope, do not update map_angle"<< std::endl;
      for(int i=1; i<2; i++){
	if (sfmEngine.Adjust(map_angle, std::make_pair(m, 1)))
	  std::cout<<"iteration: "<< i<<" done."<<std::endl;
	else return EXIT_FAILURE;
      }
       -- Export to disk computed scene (data & visualizable results)
      std::cout << "...Export SfM_Data to disk." << std::endl;
      Save(sfmEngine.Get_SfM_Data(),
	stlplus::create_filespec(sOutDir, "sfm_data", ".bin"),
	ESfM_Data(ALL));
      Save(sfmEngine.Get_SfM_Data(),
	stlplus::create_filespec(sOutDir, "sfm_data", ".json"),
	ESfM_Data(ALL));

      Save(sfmEngine.Get_SfM_Data(),
	stlplus::create_filespec(sOutDir, "cloud_and_poses", ".ply"),
	ESfM_Data(ALL));
      return EXIT_SUCCESS;
    }
    
    if (sfmEngine.Adjust(map_angle, std::make_pair(m, 1)))
    {
      std::cout << std::endl << " G2O_BA took (s): " << timer.elapsed() << std::endl;

      std::cout << "...Generating SfM_Report.html" << std::endl;
      Generate_SfM_Report(sfmEngine.Get_SfM_Data(),
	stlplus::create_filespec(sOutDir, "SfMReconstruction_Report.html"));

      //-- Export to disk computed scene (data & visualizable results)
      std::cout << "...Export SfM_Data to disk." << std::endl;
      Save(sfmEngine.Get_SfM_Data(),
	stlplus::create_filespec(sOutDir, "sfm_data", ".bin"),
	ESfM_Data(ALL));
      std::string str = std::to_string(m);
      Save(sfmEngine.Get_SfM_Data(),
	stlplus::create_filespec(sOutDir, "sfm_data"+str, ".json"),
	ESfM_Data(ALL));

      Save(sfmEngine.Get_SfM_Data(),
	stlplus::create_filespec(sOutDir, "cloud_and_poses", ".ply"),
	ESfM_Data(ALL));
      EXIT_SUCCESS;
    }
    return EXIT_FAILURE;
  }
 */

