// Copyright (c) 2018 Jessica Kang.


#include "sfm_data_BA.hpp"

#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/types.hpp"
#include "third_party/htmlDoc/htmlDoc.hpp"

// for std
#include <iostream>
// for opencv 
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <boost/concept_check.hpp>
// for g2o
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/types/slam3d/se3quat.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
// for information Matrix
#include <math.h>
#define PI 3.14159265

using namespace std;
using namespace openMVG; 
using namespace openMVG::sfm; 

using namespace openMVG::cameras;
using namespace openMVG::geometry;

//* Information matrix constructor *//
Eigen::Matrix2d info_mat(double angle, double m, double n)
{
   /* construct rotation matrix (b stands for the angle for rotation)
    * R = 
    * | cosb  -sinb |
    * | sinb   cosb |
    */
   double b;
   if (angle > 0){
     b = PI - angle;
   }
   else {
     b = angle * (-1);
   }
   Eigen::Matrix2d R;
   R << cos(b), (-1) * sin(b), sin(b), cos(b);     
   
   /* construct covariance matrix
    * C = 
    * R^t * | m  0 | * 1.0/(m+n) * R
    *       | 0  n |
    */
   Eigen::Matrix2d tmp;
   tmp << m*1.0/(m*n), 0, 0, n*1.0/(m*n);
   Eigen::Matrix2d C = R.transpose() * tmp * R;
   
   /* construct information matrix
    * I = C^-1
    */
   Eigen::Matrix2d I = C.inverse();
   
   return I;
   
}

bool Bundle_Adjustment_G2O::Adjust(SfM_Data& sfm_data, bool bVerbose, bool kernel, int iteration, map <pair<IndexT, pair<double, double>>, double> map_angle, pair<double, double> param_pair)
{
    
  g2o::SparseOptimizer  optimizer;
  g2o::BlockSolver_6_3::LinearSolverType* linearSolver = new  g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType> ();
  g2o::BlockSolver_6_3* block_solver = new g2o::BlockSolver_6_3( linearSolver );
  g2o::OptimizationAlgorithmLevenberg* algorithm = new g2o::OptimizationAlgorithmLevenberg( block_solver );
  
  optimizer.setAlgorithm( algorithm );
  optimizer.setVerbose( false );
  
  // set up vertex & parameter
  int pose_count = sfm_data.poses.size();  
  // extrinstic
  for (Poses::const_iterator itPose = sfm_data.poses.begin(); 
       itPose != sfm_data.poses.end(); ++itPose)
       {
	  const IndexT indexPose = itPose->first;

	  const Pose3 & pose = itPose->second;
	  Mat3 R = pose.rotation();
	  Vec3 t = pose.translation();
	  
	  g2o::VertexSE3Expmap* v = new g2o::VertexSE3Expmap();
	  v->setId(indexPose);
	  
	  v->setEstimate( g2o::SE3Quat(R, t) );
	  optimizer.addVertex( v );
	}  
  
  // structure
  for (Landmarks::iterator iterTracks = sfm_data.structure.begin();
    iterTracks!= sfm_data.structure.end(); ++iterTracks)
       {
	  const IndexT indexPoint = iterTracks->first;
	  Vec3 p = iterTracks->second.X;
	  
	  g2o::VertexSBAPointXYZ* v = new g2o::VertexSBAPointXYZ();
	  v->setId(pose_count + indexPoint);
	  
	  v->setMarginalized(true);
	  v->setEstimate( p );
	  optimizer.addVertex( v );
	}
  
  // intrinstic
  for (Intrinsics::const_iterator itIntrinsic = sfm_data.intrinsics.begin();
    itIntrinsic != sfm_data.intrinsics.end(); ++itIntrinsic)
       { 
	  const IndexT indexCam = itIntrinsic->first;
	  std::vector<double> intrinsic = itIntrinsic->second->getParams();
	  double focal = intrinsic[0];
	  Vec2 principal_point = Vec2(intrinsic[1], intrinsic[2]);
	  g2o::CameraParameters* camera = new g2o::CameraParameters( focal, principal_point, 0 );
	  camera->setId(indexCam);
	  optimizer.addParameter( camera );
	}
  
  // set up edge 
  vector<g2o::EdgeProjectXYZ2UV*> edges;
  for (Landmarks::iterator iterTracks = sfm_data.structure.begin();
       iterTracks!= sfm_data.structure.end(); ++iterTracks)
       {
	 const IndexT indexPoint = iterTracks->first;
	 const Observations & obs = iterTracks->second.obs;
	 for (Observations::const_iterator itObs = obs.begin();
	      itObs != obs.end(); ++itObs)
	      {
		const View * view = sfm_data.views.at(itObs->first).get();
		const IndexT indexCam = view->id_intrinsic;
		const IndexT indexPose = view->id_pose;
		Vec2 x = itObs->second.x;
		g2o::EdgeProjectXYZ2UV*  edge = new g2o::EdgeProjectXYZ2UV();
		edge->setVertex( 0, dynamic_cast<g2o::VertexSBAPointXYZ*>   (optimizer.vertex(indexPoint+pose_count)) );
		edge->setVertex( 1, dynamic_cast<g2o::VertexSE3Expmap*>     (optimizer.vertex(indexPose)) );
		edge->setMeasurement( x);
		
		// core algorithm here !!!
		map <pair<IndexT, pair<double, double>>, double>::iterator iter = map_angle.find(make_pair(itObs->first, make_pair(x(0, 0), x(1, 0))));
		if(iter != map_angle.end())
		{
		  Eigen::Matrix2d info = info_mat(map_angle[make_pair(itObs->first, make_pair(x(0, 0), x(1, 0)))], param_pair.first, param_pair.second);
		  edge->setInformation( Eigen::Matrix2d::Identity() * info );
		}
		else {
		  Eigen::Matrix2d info = info_mat(0, param_pair.first, param_pair.second);
		  edge->setInformation( Eigen::Matrix2d::Identity() * info );

		}
		
		
		edge->setParameterId(0,indexCam);
		// kernel function
		if (kernel)
		{
		  edge->setRobustKernel( new g2o::RobustKernelHuber() );
		}
		else
		{
		  edge->setRobustKernel( 0 );

		}
		
		
		optimizer.addEdge( edge );
		edges.push_back(edge);
	      }
       }
  
  if (bVerbose)
  {
    cout<<"[G2O] Start optimization..."<<endl;
    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(iteration);
    cout<<"[G2O] Optimization done."<<endl;
  }
  
  // update camera poses with refined data
  for (Poses::iterator itPose = sfm_data.poses.begin();
       itPose != sfm_data.poses.end(); ++itPose)
       {
	 const IndexT indexPose = itPose->first;
	 g2o::VertexSE3Expmap* v = dynamic_cast<g2o::VertexSE3Expmap*>( optimizer.vertex(indexPose) );
	 Eigen::Isometry3d refined_pose = v->estimate();
	 Mat3 R_refined = refined_pose.rotation();
	 Vec3 t_refined = refined_pose.translation();
	 // Update the pose
	 Pose3 & pose = itPose->second;
	 pose = Pose3(R_refined, -R_refined.transpose() * t_refined);
       }
	 
  // update camera intrinsics with refined data
  // TODO
  
  // update structure
  for (Landmarks::iterator iterTracks = sfm_data.structure.begin();
    iterTracks!= sfm_data.structure.end(); ++iterTracks)
       {
	 const IndexT indexPoint = iterTracks->first;
	 g2o::VertexSBAPointXYZ* v = dynamic_cast<g2o::VertexSBAPointXYZ*>( optimizer.vertex(pose_count + indexPoint) );
	 Vec3 refined_p = v->estimate();
	 // Update the structure
	 Vec3 & p = iterTracks->second.X;
	 p = refined_p;
       }
  
  return true;

}

bool Bundle_Adjustment_G2O::Adjust(SfM_Data& sfm_data, bool bVerbose, bool kernel, int iteration)
{
    
  g2o::SparseOptimizer  optimizer;
  g2o::BlockSolver_6_3::LinearSolverType* linearSolver = new  g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType> ();
  g2o::BlockSolver_6_3* block_solver = new g2o::BlockSolver_6_3( linearSolver );
  g2o::OptimizationAlgorithmLevenberg* algorithm = new g2o::OptimizationAlgorithmLevenberg( block_solver );
  
  optimizer.setAlgorithm( algorithm );
  optimizer.setVerbose( false );
  
  // set up vertex & parameter
  int pose_count = sfm_data.poses.size();  
  // extrinstic
  for (Poses::const_iterator itPose = sfm_data.poses.begin(); 
       itPose != sfm_data.poses.end(); ++itPose)
       {
	  const IndexT indexPose = itPose->first;

	  const Pose3 & pose = itPose->second;
	  Mat3 R = pose.rotation();
	  Vec3 t = pose.translation();
	  
	  g2o::VertexSE3Expmap* v = new g2o::VertexSE3Expmap();
	  v->setId(indexPose);
	  
	  v->setEstimate( g2o::SE3Quat(R, t) );
	  optimizer.addVertex( v );
	}  
  
  // structure
  for (Landmarks::iterator iterTracks = sfm_data.structure.begin();
    iterTracks!= sfm_data.structure.end(); ++iterTracks)
       {
	  const IndexT indexPoint = iterTracks->first;
	  Vec3 p = iterTracks->second.X;
	  
	  g2o::VertexSBAPointXYZ* v = new g2o::VertexSBAPointXYZ();
	  v->setId(pose_count + indexPoint);
	  
	  v->setMarginalized(true);
	  v->setEstimate( p );
	  optimizer.addVertex( v );
	}
  
  // intrinstic
  for (Intrinsics::const_iterator itIntrinsic = sfm_data.intrinsics.begin();
    itIntrinsic != sfm_data.intrinsics.end(); ++itIntrinsic)
       { 
	  const IndexT indexCam = itIntrinsic->first;
	  std::vector<double> intrinsic = itIntrinsic->second->getParams();
	  double focal = intrinsic[0];
	  Vec2 principal_point = Vec2(intrinsic[1], intrinsic[2]);
	  g2o::CameraParameters* camera = new g2o::CameraParameters( focal, principal_point, 0 );
	  camera->setId(indexCam);
	  optimizer.addParameter( camera );
	}
  
  // set up edge 
  vector<g2o::EdgeProjectXYZ2UV*> edges;
  for (Landmarks::iterator iterTracks = sfm_data.structure.begin();
       iterTracks!= sfm_data.structure.end(); ++iterTracks)
       {
	 const IndexT indexPoint = iterTracks->first;
	 const Observations & obs = iterTracks->second.obs;
	 for (Observations::const_iterator itObs = obs.begin();
	      itObs != obs.end(); ++itObs)
	      {
		const View * view = sfm_data.views.at(itObs->first).get();
		const IndexT indexCam = view->id_intrinsic;
		const IndexT indexPose = view->id_pose;
		Vec2 x = itObs->second.x;
		g2o::EdgeProjectXYZ2UV*  edge = new g2o::EdgeProjectXYZ2UV();
		edge->setVertex( 0, dynamic_cast<g2o::VertexSBAPointXYZ*>   (optimizer.vertex(indexPoint+pose_count)) );
		edge->setVertex( 1, dynamic_cast<g2o::VertexSE3Expmap*>     (optimizer.vertex(indexPose)) );
		edge->setMeasurement( x);
		edge->setInformation( Eigen::Matrix2d::Identity());
		edge->setParameterId(0,indexCam);
		// kernel function
		if (kernel)
		{
		  edge->setRobustKernel( new g2o::RobustKernelHuber() );
		}
		else
		{
		  edge->setRobustKernel( 0 );

		}
		
		
		optimizer.addEdge( edge );
		edges.push_back(edge);
	      }
       }
  
  if (bVerbose)
  {
    cout<<"[G2O] Start optimization..."<<endl;
    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(iteration);
    cout<<"[G2O] Optimization done."<<endl;
  }
  
  // update camera poses with refined data
  for (Poses::iterator itPose = sfm_data.poses.begin();
       itPose != sfm_data.poses.end(); ++itPose)
       {
	 const IndexT indexPose = itPose->first;
	 g2o::VertexSE3Expmap* v = dynamic_cast<g2o::VertexSE3Expmap*>( optimizer.vertex(indexPose) );
	 Eigen::Isometry3d refined_pose = v->estimate();
	 Mat3 R_refined = refined_pose.rotation();
	 Vec3 t_refined = refined_pose.translation();
	 // Update the pose
	 Pose3 & pose = itPose->second;
	 pose = Pose3(R_refined, -R_refined.transpose() * t_refined);
       }
	 
  // update camera intrinsics with refined data
  // TODO
  
  // update structure
  for (Landmarks::iterator iterTracks = sfm_data.structure.begin();
    iterTracks!= sfm_data.structure.end(); ++iterTracks)
       {
	 const IndexT indexPoint = iterTracks->first;
	 g2o::VertexSBAPointXYZ* v = dynamic_cast<g2o::VertexSBAPointXYZ*>( optimizer.vertex(pose_count + indexPoint) );
	 Vec3 refined_p = v->estimate();
	 // Update the structure
	 Vec3 & p = iterTracks->second.X;
	 p = refined_p;
       }
  
  return true;

}

