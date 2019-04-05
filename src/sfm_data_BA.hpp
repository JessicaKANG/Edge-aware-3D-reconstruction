// Copyright (c) 2018 Jessica Kang.

#ifndef SFM_DATA_BA_G2O_HPP
#define SFM_DATA_BA_G2O_HPP


#include "openMVG/sfm/sfm_data.hpp"

using namespace std;
using namespace openMVG;
using namespace sfm;

//struct SfM_Data;

struct BA_G2O_options{
      bool bVerbose_;
      int iteration_;
      //Extrinsic_Parameter_Type extrinsics_opt;
      //bool robustify_;
};

class Bundle_Adjustment_G2O{
  
  public:
    
    bool Adjust(
      // the SfM scene to refine
      SfM_Data & sfm_data,
      // tell which parameter needs to be adjusted
      //const BA_G2O_options options
      bool bVerbose_,
      bool kernel_,
      int iteration_,
      // params for core algorithm
      map <pair<IndexT, pair<double, double>>, double> map_angle,
      pair<double, double> param_pair
    );
    bool Adjust(
      // the SfM scene to refine
      SfM_Data & sfm_data,
      // tell which parameter needs to be adjusted
      //const BA_G2O_options options
      bool bVerbose_,
      bool kernel_,
      int iteration_
    );
    
};

#endif // SFM_DATA_BA_G2OHPP
