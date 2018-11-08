#include <random>
#include <iostream>
#include <fstream>
#include <sstream>
#include "ROAMestimation/ROAMestimation.h"

using namespace std;
using namespace ROAMestimation;

const static int _OFF = -1;


void getMeasurement(double &time,Eigen::VectorXd &zw,string s)
{
  
    istringstream ss( s );
    string tmp;
    getline( ss, tmp, ',' );
    time = stod(tmp);
    getline( ss, tmp, ',' );
    zw[0]=stod(tmp);
    getline( ss, tmp, ',' );
    zw[1]=stod(tmp);
    getline( ss, tmp, ',' );
    zw[2]=stod(tmp);
    getline( ss, tmp, ',' );
    zw[3]=stod(tmp);
  
  
}





int main(int argc, char *argv[]) {


  
  /* ---------------------- Set solver properties ---------------------- */

  FactorGraphFilter *f = FactorGraphFilterFactory::getNewFactorGraphFilter();
  f->setDeadReckoning(false); // first pose is fixed
  f->setSolverMethod(GaussNewton);

  int ret = system("mkdir /tmp/roamfree");
  f->setLowLevelLogging(true); // default log folder
  f->setWriteGraph(true);
  f->setWriteHessianStructure(true);

  /* ---------------------- Configure sensors ---------------------- */
  
  // Vehicle Dynamic Model
  
  Eigen::VectorXd R_OS_VDM(7); // Transformation between Accelerometer and robot frame
  R_OS_VDM << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;
  
  f->addSensor("VDM", QuadDynamicModel, true, true); // master sensor, sequential sensor
  f->setSensorFrame("VDM", R_OS_VDM);
  
  //Wind parameter
  Eigen::VectorXd wndParams(3); // Initial parameters for Wind
  wndParams << 0.0, 0.0, 0.0;

  f->addConstantParameter(Euclidean3D, "VDM_Wnd", wndParams, true);

  //Drag parameter
  Eigen::VectorXd dragParams(3); // Initial parameters for Drag
  dragParams << 0000.1,0000.1, 0000.2;  

  f->addConstantParameter(Euclidean3D, "VDM_Drag", dragParams, true);  
  
  //Mechanical Properties parameter
  Eigen::VectorXd mcParams(3); // Initial parameters for Mechanical Properties
  mcParams << 1.6e-4, 7.5e-7, 6e-7;  

  f->addConstantParameter(Euclidean3D, "VDM_Mc", mcParams, true);
  
  //Copter properties parameter
  Eigen::VectorXd cpParams(2); // Initial parameters for Copter Properties
  cpParams << 1.454, 0.40;  
  f->addConstantParameter(Euclidean2D, "VDM_Cp", cpParams, true);
  
  
  //Inertia Diagonal parameter
  Eigen::VectorXd ibdParams(3); // Initial parameters for Inertia Diagonal
  ibdParams << 5.8e-3, 6.0e-3, 1.1e-2;  

  f->addConstantParameter(Euclidean3D, "VDM_Ibd", ibdParams, true);
  
  
  //Inertia Off-Diagonal parameter
  Eigen::VectorXd ibodParams(3); // Initial parameters for Inertia Off-Diagonal
  ibodParams << 0.0, 0.0, 0.0;

  f->addConstantParameter(Euclidean3D, "VDM_Ibod", ibodParams, true);
  
  
  
  Eigen::MatrixXd vdmCov(4, 4); // covariance of Accelerometer readings
  vdmCov =  1*Eigen::MatrixXd::Identity(6, 6);
  
  Eigen::MatrixXd posCov(6, 6);
  posCov =  1*Eigen::MatrixXd::Identity(6, 6);
  
  Eigen::VectorXd x0(7), x1(7);
  x0 << 0,0,0,0.707107019200454,0.0,0,0.707107019200454;
  x1 << 0,0,0,0.707107019200454,0.0,0,0.707107019200454;
 
  double t = -0.01;
  PoseVertexWrapper_Ptr firstPose = f->setInitialPose(x0, t);
  
   
  ifstream infile( argv[1] );
  
  int cntVdm = 0;
  int vdmRate = 50;
  bool firstEstimate = true;
  double priorTime = -numeric_limits< double >::infinity();
  
  
   while (infile)
   {   
  
    if(!infile.is_open() || infile.eof())
    {
      break; 
    }
    
    try 
    {
    
      string s;
      getline( infile, s );    
      double time;
      Eigen::VectorXd zw(4);
      getMeasurement(time,zw,s);
      f->addSequentialMeasurement("VDM", time, zw, vdmCov);

    }
    catch (const std::invalid_argument& e)
    {
      cout<< e.what() << endl;
      break;
      
    }
    
    // Add Measurement
    if(cntVdm==0)
    {
      f->getNewestPose()->setEstimate(x1);
    }
    else
    {     
      const Eigen::VectorXd &p1 =  f->getNthPose(1)->getEstimate();
      const Eigen::VectorXd &p2 =  f->getNthPose(2)->getEstimate();
      Eigen::VectorXd predcitedPose(7);
      predcitedPose.head(3) = p1.head(3)+(p1.head(3)-p2.head(3));
      predcitedPose.tail(4) = p1.tail(4);
//       f->getNewestPose()->setEstimate(predcitedPose);
//      cerr << time << " " << predcitedPose.transpose()<<endl;
    }

    if (cntVdm > 2.0 && cntVdm % (vdmRate) == 0)
    {
      PoseVertexWrapper_Ptr oldest = f->getNthOldestPose(0);
      PoseVertexWrapper_Ptr secondoldest = f->getNthOldestPose(1);
  
      if(firstEstimate)
      {
	
	firstEstimate = false;
	oldest->setEstimate(x0);
	secondoldest->setEstimate(x1); 
	
      }     
      
      
      
      if(oldest->getTimestamp() > priorTime)
      {
	f->addPriorOnPose(oldest,oldest->getEstimate(),posCov);    
	f->addPriorOnPose(secondoldest,secondoldest->getEstimate(),posCov);
	priorTime = oldest->getTimestamp();
	
      }
      
      
      cerr << oldest->getTimestamp() << " " << oldest->getEstimate().transpose() <<endl;
      cerr <<  secondoldest->getTimestamp() << " " << secondoldest->getEstimate().transpose() <<endl;
      
      if(!f->estimate(10))
	return 1;
      
       f->forgetOldNodes(2.5);
       
    }
    
    cntVdm++;
    
   }
  
  
  
 return 0;

}