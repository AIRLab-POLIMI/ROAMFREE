// g2o - General Graph Optimization
// Copyright (C) 2011 H. Strasdat
//
// g2o is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// g2o is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include <Eigen/StdVector>
#include <tr1/random>
#include <iostream>
#include <stdint.h>
#include <tr1/unordered_set>

#include "g2o/core/graph_optimizer_sparse.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/types/icp/types_icp.h"
#include "g2o/core/structure_only_solver.h"

using namespace Eigen;
using namespace std;


class Sample
{

  static tr1::ranlux_base_01 gen_real;
  static tr1::mt19937 gen_int;
public:
  static int uniform(int from, int to);

  static double uniform();

  static double gaussian(double sigma);
};


tr1::ranlux_base_01 Sample::gen_real;
tr1::mt19937 Sample::gen_int;

int Sample::uniform(int from, int to)
{
  tr1::uniform_int<int> unif(from, to);
  int sam = unif(gen_int);
  return  sam;
}

double Sample::uniform()
{
  std::tr1::uniform_real<double> unif(0.0, 1.0);
  double sam = unif(gen_real);
  return  sam;
}

double Sample::gaussian(double sigma)
{
  std::tr1::normal_distribution<double> gauss(0.0, sigma);
  double sam = gauss(gen_real);
  return  sam;
}

int main(int argc, const char* argv[])
{
  if (argc<2)
  {
    cout << endl;
    cout << "Please type: " << endl;
    cout << "ba_demo [PIXEL_NOISE] [OUTLIER RATIO] [ROBUST_KERNEL] [STRUCTURE_ONLY] [DENSE]" << endl;
    cout << endl;
    cout << "PIXEL_NOISE: noise in image space (E.g.: 1)" << endl;
    cout << "OUTLIER_RATIO: probability of spuroius observation  (default: 0.0)" << endl;
    cout << "ROBUST_KERNEL: use robust kernel (0 or 1; default: 0==false)" << endl;
    cout << "STRUCTURE_ONLY: performe structure-only BA to get better point initializations (0 or 1; default: 0==false)" << endl;
    cout << "DENSE: Use dense solver (0 or 1; default: 0==false)" << endl;
    cout << endl;
    cout << "Note, if OUTLIER_RATIO is above 0, ROBUST_KERNEL should be set to 1==true." << endl;
    cout << endl;
    exit(0);
  }

  double PIXEL_NOISE = atof(argv[1]);

  double OUTLIER_RATIO = 0.0;

  if (argc>2)
  {
    OUTLIER_RATIO = atof(argv[2]);
  }

  bool ROBUST_KERNEL = false;
  if (argc>3)
  {
    ROBUST_KERNEL = atof(argv[3]);
  }
  bool STRUCTURE_ONLY = false;
  if (argc>4)
  {
    STRUCTURE_ONLY = atof(argv[4]);
  }

  bool DENSE = false;
  if (argc>5)
  {
    DENSE = atof(argv[5]);
  }

  cout << "PIXEL_NOISE: " <<  PIXEL_NOISE << endl;
  cout << "OUTLIER_RATIO: " << OUTLIER_RATIO<<  endl;
  cout << "ROBUST_KERNEL: " << ROBUST_KERNEL << endl;
  cout << "STRUCTURE_ONLY: " << STRUCTURE_ONLY<< endl;
  cout << "DENSE: "<<  DENSE << endl;



  g2o::SparseOptimizer optimizer;
  optimizer.setMethod(g2o::SparseOptimizer::LevenbergMarquardt);
  optimizer.setVerbose(false);
  g2o::BlockSolver_6_3::LinearSolverType * linearSolver;
  if (DENSE)
  {
        linearSolver= new g2o::LinearSolverDense<g2o
        ::BlockSolver_6_3::PoseMatrixType>();
  }
  else
  {
    linearSolver
        = new g2o::LinearSolverCholmod<g2o
        ::BlockSolver_6_3::PoseMatrixType>();
  }


  g2o::BlockSolver_6_3 * solver_ptr
      = new g2o::BlockSolver_6_3(&optimizer,linearSolver);


  optimizer.setSolver(solver_ptr);


  // set up 500 points
  vector<Vector3d> true_points;
  for (size_t i=0;i<500; ++i)
  {
    true_points.push_back(Vector3d((Sample::uniform()-0.5)*3,
                                   Sample::uniform()-0.5,
                                   Sample::uniform()+10));
  }


  Vector2d focal_length(500,500); // pixels
  Vector2d principal_point(320,240); // 640x480 image
  double baseline = 0.075;      // 7.5 cm baseline


  vector<g2o::SE3Quat,
      aligned_allocator<g2o::SE3Quat> > true_poses;

  // set up camera params
  g2o::VertexSCam::setKcam(focal_length[0],focal_length[1],
                           principal_point[0],principal_point[1],
                           baseline);
  
  // set up 5 vertices, first 2 fixed
  int vertex_id = 0;
  for (size_t i=0; i<5; ++i)
  {


    Vector3d trans(i*0.04-1.,0,0);

    Eigen:: Quaterniond q;
    q.setIdentity();
    g2o::SE3Quat pose(q,trans);


    g2o::VertexSCam * v_se3
        = new g2o::VertexSCam();

    v_se3->setId(vertex_id);
    v_se3->estimate() = pose;
    v_se3->setAll();            // set aux transforms

    if (i<2)
      v_se3->setFixed(true);
    
    optimizer.addVertex(v_se3);
    true_poses.push_back(pose);
    vertex_id++;
  }

  int point_id=vertex_id;
  int point_num = 0;
  double sum_diff2 = 0;

  cout << endl;
  tr1::unordered_map<int,int> pointid_2_trueid;
  tr1::unordered_set<int> inliers;

  // add point projections to this vertex
  for (size_t i=0; i<true_points.size(); ++i)
  {
    g2o::VertexPointXYZ * v_p
        = new g2o::VertexPointXYZ();


    v_p->setId(point_id);
    v_p->setMarginalized(true);
    v_p->estimate() = true_points.at(i)
        + Vector3d(Sample::gaussian(1),
                   Sample::gaussian(1),
                   Sample::gaussian(1));



    int num_obs = 0;

    for (size_t j=0; j<true_poses.size(); ++j)
    {
      Vector3d z;
      dynamic_cast<g2o::VertexSCam*>
        (optimizer.vertices().find(j)->second)
        ->mapPoint(z,true_points.at(i));

      if (z[0]>=0 && z[1]>=0 && z[0]<640 && z[1]<480)
      {
        ++num_obs;
      }
    }

    if (num_obs>=2)
    {

      bool inlier = true;
      for (size_t j=0; j<true_poses.size(); ++j)
      {
        Vector3d z;
        dynamic_cast<g2o::VertexSCam*>
          (optimizer.vertices().find(j)->second)
          ->mapPoint(z,true_points.at(i));

        if (z[0]>=0 && z[1]>=0 && z[0]<640 && z[1]<480)
        {
          double sam = Sample::uniform();
          if (sam<OUTLIER_RATIO)
          {
            z = Vector3d(Sample::uniform(64,640),
                         Sample::uniform(0,480),
                         Sample::uniform(0,64)); // disparity
            z(2) = z(0) - z(2); // px' now

            inlier= false;
          }

          z += Vector3d(Sample::gaussian(PIXEL_NOISE),
                        Sample::gaussian(PIXEL_NOISE),
                        Sample::gaussian(PIXEL_NOISE/16.0));

          g2o::Edge_XYZ_VSC * e
              = new g2o::Edge_XYZ_VSC();


          e->vertices()[0]
              = dynamic_cast<g2o::OptimizableGraph::Vertex*>(v_p);

          e->vertices()[1]
              = dynamic_cast<g2o::OptimizableGraph::Vertex*>
              (optimizer.vertices().find(j)->second);

          e->measurement() = z;
          e->inverseMeasurement() = -z;
          e->information() = Matrix3d::Identity();

          e->setRobustKernel(ROBUST_KERNEL);
          e->setHuberWidth(1);

          optimizer.addEdge(e);


        }

      }

      if (inlier)
      {
        inliers.insert(point_id);
        Vector3d diff = v_p->estimate() - true_points[i];

        sum_diff2 += diff.dot(diff);
      }
     // else
     //   cout << "Point: " << point_id <<  "has at least one spurious observation" <<endl;


      optimizer.addVertex(v_p);

      pointid_2_trueid.insert(make_pair(point_id,i));

      ++point_id;
      ++point_num;


    }

  }


  cout << endl;
  optimizer.initializeOptimization();

  optimizer.setVerbose(true);


  g2o::StructureOnlySolver<3> structure_only_ba;

  if (STRUCTURE_ONLY)
  {
    cout << "Performing structure-only BA:"   << endl;
    structure_only_ba.setVerbose(true);
    structure_only_ba.calc(optimizer.vertices(),
                           10);


  }

    cout << endl;
  cout << "Performing full BA:" << endl;
  optimizer.optimize(10);

  cout << endl;
  cout << "Point error before optimisation (inliers only): " << sqrt(sum_diff2/point_num) << endl;


  point_num = 0;
  sum_diff2 = 0;


  for (tr1::unordered_map<int,int>::iterator it=pointid_2_trueid.begin();
       it!=pointid_2_trueid.end(); ++it)
  {

    g2o::HyperGraph::VertexIDMap::iterator v_it
        = optimizer.vertices().find(it->first);

    if (v_it==optimizer.vertices().end())
    {
      cerr << "Vertex " << it->first << " not in graph!" << endl;
      exit(-1);
    }

    g2o::VertexPointXYZ * v_p
        = dynamic_cast< g2o::VertexPointXYZ * > (v_it->second);

    if (v_p==0)
    {
      cerr << "Vertex " << it->first << "is not a PointXYZ!" << endl;
      exit(-1);
    }

    Vector3d diff = v_p->estimate()-true_points[it->second];

    if (inliers.find(it->first)==inliers.end())
      continue;

    sum_diff2 += diff.dot(diff);

    ++point_num;
  }

  cout << "Point error after optimisation (inliers only): " << sqrt(sum_diff2/point_num) << endl;
  cout << endl;

}
