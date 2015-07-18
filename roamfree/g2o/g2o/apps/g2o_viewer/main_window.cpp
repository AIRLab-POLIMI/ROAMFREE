// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
//
// This file is part of g2o.
// 
// g2o is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// g2o is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with g2o.  If not, see <http://www.gnu.org/licenses/>.

#include "main_window.h"

#include "g2o/core/factory.h"
#include "g2o/core/solver_property.h"
#include "g2o/core/solver_factory.h"
#include "g2o/core/graph_optimizer_sparse.h"
#include "g2o/core/estimate_propagator.h"
#include "g2o/core/viewer_vertex_decorator.h"

#include <QFileDialog>
#include <QStandardItemModel>
#include <QDoubleValidator>
#include <fstream>
#include <iostream>

#include <map>

using namespace std;
using namespace g2o;

MainWindow::MainWindow(QWidget * parent, Qt::WindowFlags flags) :
  QMainWindow(parent, flags),
  _lastSolver(-1)
{
  setupUi(this);
  leLambda->setValidator(new QDoubleValidator(-numeric_limits<double>::max(), numeric_limits<double>::max(), 7, this));
  leKernelWidth->setValidator(new QDoubleValidator(-numeric_limits<double>::max(), numeric_limits<double>::max(), 7, this));
  plainTextEdit->setMaximumBlockCount(1000);
  btnForceStop->hide();
}

MainWindow::~MainWindow()
{
}

void MainWindow::on_actionLoad_triggered(bool)
{
  QString filename = QFileDialog::getOpenFileName(this, "Load g2o file", "", "g2o files (*.g2o);;All Files (*)");
  if (! filename.isNull()) {
    loadFromFile(filename);
  }
}

void MainWindow::on_actionSave_triggered(bool)
{
  QString filename = QFileDialog::getSaveFileName(this, "Save g2o file", "", "g2o files (*.g2o)");
  if (! filename.isNull()) {
    ofstream fout(filename.toStdString().c_str());
    viewer->graph->save(fout);
    if (fout.good())
      cerr << "Saved " << filename.toStdString() << endl;
    else
      cerr << "Error while saving file" << endl;
  }
}

void MainWindow::on_btnDump_clicked() {

  std::fstream f;

  f.open("dump.dat", std::fstream::out);

  const HyperGraph::VertexIDMap &v_map = viewer->graph->vertices();
  for (HyperGraph::VertexIDMap::const_iterator it = v_map.begin(); it != v_map.end(); ++it) {

    g2o::OptimizableGraph::Vertex *v = static_cast<g2o::OptimizableGraph::Vertex *>(it->second);

    double *unc =  v->uncertaintyData();
    int dim = v->dimension();

    double e[dim];

    v->getEstimateData(e);

    f << v->id() << " ";

    for (int c = 0; c<dim; c++) {
      f << std::setprecision(8) << e[c] << " ";
    }

    for (int r = 0; r<dim; r++) {
      for (int c = 0; c<dim; c++) {
        f << std::setprecision(8) << unc[r*dim+c] << " ";
      }
    }

    f << std::endl;
  }

  f.close();
}

void MainWindow::on_btnOptimize_clicked()
{
  if (viewer->graph->vertices().size() == 0 || viewer->graph->edges().size() == 0) {
    cerr << "Graph has no vertices / egdes" << endl;
    return;
  }

  // fix the vertices according to checkboxes

  bool fixedSetChanged = false;

  const HyperGraph::VertexIDMap &v_map = viewer->graph->vertices();
  for (HyperGraph::VertexIDMap::const_iterator it = v_map.begin(); it != v_map.end(); ++it) {
    g2o::OptimizableGraph::Vertex *v = static_cast<g2o::OptimizableGraph::Vertex *>(it->second);
    g2o::ViewerVertexDecorator *vd = dynamic_cast<g2o::ViewerVertexDecorator *>(it->second);

    bool toset;

    if (vd != NULL) { // we work only with decorated vertices

      // this vertex has its dedicated checkbox
      if (vd->displayEstimate()) {
        //iterate trought the vector
        for (unsigned int k = 0; k<trackedVertices.size(); k++) {
          if (trackedVertices[k].first == v) {
            toset = trackedVertices[k].second->checkState(0) == Qt::Checked;
          }
        }
      } else {
        // check for the whole category
        toset = vertexTypes[vd->accessCategory()]->checkState(0) == Qt::Checked;
      }
    }

    if (toset != v->fixed()) {
      fixedSetChanged = true;
      v->setFixed(toset);
    }
  }

  if (fixedSetChanged) {
    cerr << "Set of fixed vertices has changed" << endl;
  }


  bool allocatedNewSolver;
  bool allocateStatus = allocateSolver(allocatedNewSolver);
  if (! allocateStatus) {
    cerr << "Error while allocating solver" << endl;
    return;
  }
  if (allocatedNewSolver || fixedSetChanged) {
    prepare();
  }
  setRobustKernel();

  if (rbGauss->isChecked())
    viewer->graph->setMethod(g2o::SparseOptimizer::GaussNewton);
  else if (rbLevenberg->isChecked()) {
    double lambdaInit = leLambda->text().toDouble();
    if (lambdaInit > 0) {
      cerr << "Using initial damping of " << lambdaInit << endl;
      viewer->graph->setUserLambdaInit(lambdaInit);
    } else
      viewer->graph->setUserLambdaInit(0.);
    viewer->graph->setMethod(g2o::SparseOptimizer::LevenbergMarquardt);
  } else
    viewer->graph->setMethod(g2o::SparseOptimizer::GaussNewton);

  // set the information scaling factors

  int row = EdgesCoeffs->rowCount();
  for (int k = 0; k<row; k++) {
    string edgename = EdgesCoeffs->item(k,0)->text().toStdString();
    double coeff = EdgesCoeffs->item(k,1)->text().toDouble();

    // cycle through all edges and scale the noise covariance matrices

    Factory *f = Factory::instance();

    SparseOptimizer* optimizer = viewer->graph;
    for (SparseOptimizer::EdgeSet::const_iterator it = optimizer->edges().begin(); it != optimizer->edges().end(); ++it) {

      OptimizableGraph::Edge* e = static_cast<OptimizableGraph::Edge*>(*it);
      string tag = f->tag(e);

      // TODO: buggy! if the JErr_noises is not the identity this is useless
      if ( tag == edgename ) {
        Eigen::Map<Eigen::MatrixXd> information(e->informationData(), e->dimension(), e->dimension());
        information *= coeff;
      }
    }
  }


  // ---

  btnOptimize->hide();
  btnForceStop->show();

  _forceStopFlag = false;
  viewer->graph->setForceStopFlag(&_forceStopFlag);

  int maxIterations = spIterations->value();
  int iter = viewer->graph->optimize(maxIterations);
  if (maxIterations > 0 && !iter){
    cerr << "Optimization failed, result might be invalid" << endl;
  }

  btnOptimize->show();
  btnForceStop->hide();

  // once finished show vertices values

  viewer->graph->computeMarginals();

  for (unsigned int k = 0; k < trackedVertices.size(); k++) {

    int size = trackedVertices[k].first->estimateDimension();
    double *est = new double[size];
    trackedVertices[k].first->getEstimateData(est);

    double *unc = trackedVertices[k].first->uncertaintyData();

    for (int j = 0; j < trackedVertices[k].first->estimateDimension(); j++) {
      trackedVertices[k].second->child(j)->setText(1, QString::number(est[j],'f',3));

      // uncertainty
      for (int h = 0; h < trackedVertices[k].first->estimateDimension(); h++) {
        trackedVertices[k].second->child(j)->setText(h+2,QString::number(unc[size*j+h],'f',3));
      }
    }

    delete est;
  }

  // resize columns

  for (int h = 0; h < VertexValues->columnCount(); h++) {
    VertexValues->resizeColumnToContents(h);
  }

  //

  viewer->setUpdateDisplay(true);
  viewer->updateGL();
  _forceStopFlag = false;
}

void MainWindow::on_btnInitialGuess_clicked()
{
  if (viewer->graph->activeEdges().size() == 0)
    viewer->graph->initializeOptimization();

  viewer->graph->computeInitialGuess();
  viewer->setUpdateDisplay(true);
  viewer->updateGL();
}

void MainWindow::fixGraph()
{
  if (viewer->graph->vertices().size() == 0 || viewer->graph->edges().size() == 0) {
    return;
  }

  if (false) { //ROAMFREE - we don't want this
    // check for vertices to fix to remove DoF
    bool gaugeFreedom = viewer->graph->gaugeFreedom();
    g2o::OptimizableGraph::Vertex* gauge = viewer->graph->findGauge();
    if (gaugeFreedom) {
      if (! gauge) {
        cerr <<  "cannot find a vertex to fix in this thing" << endl;
        return;
      } else {
        cerr << "graph is fixed by node " << gauge->id() << endl;
        gauge->setFixed(true);
      }
    } else {
      cerr << "graph is fixed by priors" << endl;
  }
  }

  viewer->graph->setVerbose(true);
  //viewer->graph->computeActiveErrors();
}

void MainWindow::on_actionQuit_triggered(bool)
{
  close();
}

void MainWindow::updateDisplayedSolvers()
{
  const SolverFactory::CreatorList& knownSolvers = SolverFactory::instance()->creatorList();

  bool varFound = false;
  string varType = "";
  for (SolverFactory::CreatorList::const_iterator it = knownSolvers.begin(); it != knownSolvers.end(); ++it) {
    const SolverProperty& sp = (*it)->property();
    if (sp.name == "var" || sp.name == "var_cholmod") {
      varType = sp.type;
      varFound = true;
      break;
    }
  }

  if (varFound) {
    for (SolverFactory::CreatorList::const_iterator it = knownSolvers.begin(); it != knownSolvers.end(); ++it) {
      const SolverProperty& sp = (*it)->property();
      if (sp.type == varType) {
        coOptimizer->addItem(QString::fromStdString(sp.name));
        _knownSolvers.push_back(sp);
      }
    }
  }

  map<string, vector<SolverProperty> > solverLookUp;

  for (SolverFactory::CreatorList::const_iterator it = knownSolvers.begin(); it != knownSolvers.end(); ++it) {
    const SolverProperty& sp = (*it)->property();
    if (varFound && varType == sp.type)
      continue;
    solverLookUp[sp.type].push_back(sp); 
  }

  for (map<string, vector<SolverProperty> >::iterator it = solverLookUp.begin(); it != solverLookUp.end(); ++it) {
    if (_knownSolvers.size() > 0) {
      coOptimizer->insertSeparator(coOptimizer->count());
      _knownSolvers.push_back(SolverProperty());
    }
    const vector<SolverProperty>& vsp = it->second;
    for (size_t j = 0; j < vsp.size(); ++j) {
      coOptimizer->addItem(QString::fromStdString(vsp[j].name));
      _knownSolvers.push_back(vsp[j]);
    }
  }
}

bool MainWindow::load(const QString& filename)
{
  ifstream ifs(filename.toStdString().c_str());
  if (! ifs)
    return false;
  viewer->graph->clear();
  bool loadStatus = viewer->graph->load(ifs);
  if (! loadStatus)
    return false;
  _lastSolver = -1;
  viewer->setUpdateDisplay(true);
  SparseOptimizer* optimizer = viewer->graph;

  // update the solvers which are suitable for this graph
  set<int> vertDims = optimizer->dimensions();
  for (size_t i = 0; i < _knownSolvers.size(); ++i) {
    const SolverProperty& sp = _knownSolvers[i];
    if (sp.name == "" && sp.desc == "")
      continue;

    bool suitableSolver = optimizer->isSolverSuitable(sp, vertDims);
    qobject_cast<QStandardItemModel *>(coOptimizer->model())->item(i)->setEnabled(suitableSolver);
  }
  return loadStatus;
}

bool MainWindow::allocateSolver(bool& allocatedNewSolver)
{
  if (coOptimizer->count() == 0) {
    cerr << "No solvers available" << endl;
    return false;
  }
  int currentIndex = coOptimizer->currentIndex();
  bool enabled = qobject_cast<QStandardItemModel *>(coOptimizer->model())->item(currentIndex)->isEnabled();

  if (! enabled) {
    cerr << "selected solver is not enabled" << endl;
    return false;
  }

  if (currentIndex == _lastSolver)
    return true;

  allocatedNewSolver = true;
  QString strSolver = coOptimizer->currentText();
  delete viewer->graph->solver();

  SolverFactory* solverFactory = SolverFactory::instance();
  viewer->graph->setSolver(solverFactory->construct(strSolver.toStdString(), viewer->graph, _currentSolverProperty));

  _lastSolver = currentIndex;
  return true;
}

bool MainWindow::prepare()
{
  SparseOptimizer* optimizer = viewer->graph;
  if (_currentSolverProperty.requiresMarginalize) {
    cerr << "Marginalizing Landmarks" << endl;
    for (SparseOptimizer::VertexIDMap::const_iterator it = optimizer->vertices().begin(); it != optimizer->vertices().end(); ++it) {
      OptimizableGraph::Vertex* v = static_cast<OptimizableGraph::Vertex*>(it->second);
      int vdim = v->dimension();
      v->setMarginalized((vdim == _currentSolverProperty.landmarkDim));
    }
  }
  else {
    cerr << "Preparing (no marginalization of Landmarks)" << endl;
    for (SparseOptimizer::VertexIDMap::const_iterator it = optimizer->vertices().begin(); it != optimizer->vertices().end(); ++it) {
      OptimizableGraph::Vertex* v = static_cast<OptimizableGraph::Vertex*>(it->second);
      v->setMarginalized(false);
    }
  }
  viewer->graph->initializeOptimization();
  return true;
}

void MainWindow::setRobustKernel()
{
  SparseOptimizer* optimizer = viewer->graph;
  bool robustKernel = cbRobustKernel->isChecked();
  double huberWidth = leKernelWidth->text().toDouble();

  for (SparseOptimizer::EdgeSet::const_iterator it = optimizer->edges().begin(); it != optimizer->edges().end(); ++it) {
    OptimizableGraph::Edge* e = static_cast<OptimizableGraph::Edge*>(*it);
    e->setRobustKernel(robustKernel);
    e->setHuberWidth(huberWidth);
  }
}

void MainWindow::on_btnForceStop_clicked()
{
  _forceStopFlag = true;
}

bool MainWindow::loadFromFile(const QString& filename)
{
  viewer->graph->clear();
  bool loadStatus = load(filename);
  cerr << "loaded " << filename.toStdString() << " with " << viewer->graph->vertices().size()
    << " vertices and " << viewer->graph->edges().size() << " measurments" << endl;

  // iterate trough edges and add edges types to the TableWidget

  map<std::string, double> names;
  Factory *f = Factory::instance();

  SparseOptimizer* optimizer = viewer->graph;
  for (SparseOptimizer::EdgeSet::const_iterator it = optimizer->edges().begin(); it != optimizer->edges().end(); ++it) {
    OptimizableGraph::Edge* e = static_cast<OptimizableGraph::Edge*>(*it);

    string tag = f->tag(e);
    // TODO: we should have a better mechanism to exclude the E_SE3PRIOR
    if (names.count(tag) == 0 && tag != "E_SE3PRIOR") {
      names[tag] = 1.0;
    }
  }

  // add edges types to the widget

  int row = EdgesCoeffs->rowCount();
  for (std::map<string,double>::iterator it=names.begin(); it!=names.end(); ++it) {
    EdgesCoeffs->insertRow(row);

    QTableWidgetItem* edge_name = new QTableWidgetItem();
    edge_name->setText(QString::fromStdString(it->first));

    QTableWidgetItem* edge_coeff = new QTableWidgetItem();
    edge_coeff->setText(QString::number(it->second));

    EdgesCoeffs->setItem(row,0, edge_name);
    EdgesCoeffs->setItem(row,1, edge_coeff);
  }

  EdgesCoeffs->resizeColumnsToContents();

  // find the vertices which have to be tracked

  const HyperGraph::VertexIDMap &v_map = viewer->graph->vertices();
  for (HyperGraph::VertexIDMap::const_iterator it = v_map.begin(); it != v_map.end(); ++it) {
    g2o::OptimizableGraph::Vertex *v = static_cast<g2o::OptimizableGraph::Vertex *>(it->second);
    g2o::ViewerVertexDecorator *vd = dynamic_cast<g2o::ViewerVertexDecorator *>(it->second);

    if (vd != NULL) { // we work only with decorated vertices
      if (vd->displayEstimate()) {

        QTreeWidgetItem *item = new QTreeWidgetItem(VertexValues);
        item->setCheckState(0, v->fixed() ? Qt::Checked : Qt::Unchecked);
        item->setText(1, QString::fromStdString(vd->accessCategory()));
        item->setExpanded(true);

        trackedVertices.push_back( pair<g2o::OptimizableGraph::Vertex *, QTreeWidgetItem *>(v,item) );

        int size = v->estimateDimension();
        double *est = new double[size];
        v->getEstimateData(est);

        VertexValues->setColumnCount(max(size+1, VertexValues->columnCount()));

        for (int k = 0; k<size; k++) {
          QTreeWidgetItem *item2 = new QTreeWidgetItem(item);
          item2->setText(1, QString::number(est[k],'f',3));

          // uncertainty
          for (int h = 0; h<size; h++) {
            item2->setText(h+2,QString::number(0,'f',3));
          }
        }

        delete est;
      } else {
        // if we don't have any vertex of this type in the widget
        if (vertexTypes.count(vd->accessCategory()) == 0) {
          QTreeWidgetItem *item = new QTreeWidgetItem(VertexValues);
          item->setCheckState(0, v->fixed() ? Qt::Checked : Qt::Unchecked);
          item->setText(1, QString::fromStdString(vd->accessCategory()));
          item->setExpanded(true);

          vertexTypes[vd->accessCategory()] = item;
        }
      }
    }
  }

  // resize columns

  for (int h = 0; h < VertexValues->columnCount(); h++) {
    VertexValues->resizeColumnToContents(h);
  }

  //

  viewer->updateGL();
  fixGraph();
  return loadStatus;
}

void MainWindow::on_actionWhite_Background_triggered(bool)
{
  viewer->setBackgroundColor(QColor::fromRgb(255, 255, 255));
  viewer->updateGL();
}

void MainWindow::on_actionDefault_Background_triggered(bool)
{
  viewer->setBackgroundColor(QColor::fromRgb(51, 51, 51));
  viewer->updateGL();
}

void MainWindow::on_actionSave_Screenshot_triggered(bool)
{
  QString selectedFilter;
  QString filename = QFileDialog::getSaveFileName(this, "Save screen to a file", "viewer.png",
      "PNG files (*.png);;JPG files (*.jpg);;EPS files (*.eps)", &selectedFilter);

  if (! filename.isNull()) {
    // extract the file format from the filter options
    int spacePos = selectedFilter.indexOf(' ');
    assert(spacePos > 0 && "extracting the image format failed");
    QString format = selectedFilter.left(spacePos);
    // setting up the snapshot and save to file
    if (format == "JPG") {
      viewer->setSnapshotQuality(90);
    } else {
      viewer->setSnapshotQuality(-1);
    }
    viewer->setSnapshotFormat(format);
    viewer->saveSnapshot(filename);
    cerr << "saved snapshot " << filename.toStdString() << "(" << format.toStdString() << ")" << endl;
  }
}
