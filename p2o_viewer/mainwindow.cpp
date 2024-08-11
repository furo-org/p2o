/****************************************************************************
 * p2o: Petite Portable Pose-graph Optimizer v2
 * Copyright (C) 2010-2024 Kiyoshi Irie
 * Copyright (C) 2017-2024 Future Robotics Technology Center (fuRo),
 *                         Chiba Institute of Technology.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at https://mozilla.org/MPL/2.0/.
 ****************************************************************************/
#include "mainwindow.h"
#include "load_p2o_file.h"
#include <vtkGenericOpenGLRenderWindow.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/eigen.h>
#include <pcl/filters/voxel_grid.h>
#include <vtkPolyLine.h>
#include <QFileDialog>
#include <QMessageBox>
#include <QProgressDialog>
#include <QSettings>

MainWindow::MainWindow(QWidget *parent) :
        mPoseGraphVis("graph_in", Eigen::Vector3f(1, 1, 1)),
        mResultPoseGraphVis("graph_out", Eigen::Vector3f(1, 0, 0)),
    QMainWindow(parent) {
    loadSettings();
    ui.setupUi(this);
    connectSignalsAndSlots();
    onShowNodeOrientationCheckedChanged(ui.checkShowNodeOrientation->checkState()==Qt::Checked);
    auto renderer = vtkSmartPointer<vtkRenderer>::New();
    auto renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    renderWindow->AddRenderer(renderer);
    mViewer.reset (new pcl::visualization::PCLVisualizer(renderer, renderWindow, "PCL Viewer", false));
    mViewer->setBackgroundColor(0, 0, 0);
    mViewer->addCoordinateSystem(1.0);
#if VTK_MAJOR_VERSION < 9
    ui.qvtkWidget->SetRenderWindow(mViewer->getRenderWindow());
#else
    ui.qvtkWidget->setRenderWindow(mViewer->getRenderWindow());
#endif
    mViewer->getRenderWindow()->GlobalWarningDisplayOff();
    this->setStatusBar(&mStatusBar);
}

MainWindow::~MainWindow() {
    saveSettings();
}

void MainWindow::connectSignalsAndSlots() {
    bool res = connect(ui.actionLoad_p2o_file, SIGNAL(triggered()), this, SLOT(onLoadP2oFile()));
    assert(res);
    res = connect(ui.actionLoad_PCD_file, SIGNAL(triggered()), this, SLOT(onLoadPCDFile()));
    assert(res);
    res = connect(ui.actionSave_point_cloud_map, SIGNAL(triggered()), this, SLOT(onSavePCDFile()));
    assert(res);
    res = connect(ui.pushOptimizePoseGraph, SIGNAL(clicked()), this, SLOT(onPushOptimizePoseGraph()));
    assert(res);
    res = connect(ui.pushShowPointCloudMap, SIGNAL(clicked()), this, SLOT(onPushShowPointCloudMap()));
    assert(res);
    res = connect(ui.pushHidePointCloudMap, SIGNAL(clicked()), this, SLOT(onPushHidePointCloudMap()));
    assert(res);
    res = connect(ui.checkShowNodeOrientation, SIGNAL(stateChanged(int)), this, SLOT(onShowNodeOrientationCheckedChanged(int)));
    assert(res);
    res = connect(ui.checkShowEdges, SIGNAL(stateChanged(int)), this, SLOT(onShowEdgesCheckedChanged(int)));
    assert(res);
}

void MainWindow::onPushOptimizePoseGraph() {
    p2o::Optimizer3D opt;
    int min_steps = ui.spinMinSteps->value();
    int max_steps = ui.spinMaxSteps->value();
    opt.setRobustThreshold(ui.spinRobustThre->value());
    mResultNodes = opt.optimizePath(mNodes, mErrorFuncs, max_steps, min_steps);
    showPoseGraph(mResultNodes, mErrorFuncs, mResultPoseGraphVis);
}

void MainWindow::onPushShowPointCloudMap() {
    p2o::Pose3DVec &nodes_vis = this->mResultNodes;
    if (nodes_vis.empty()) {
        nodes_vis = mNodes;
    }
    if (nodes_vis.empty()) {
        QMessageBox::warning(this, "Error", "No pose graph loaded.");
        return;
    }
    mMapCloud.reset(new pcl::PointCloud<pcl::PointXYZI>);

    QProgressDialog progress("Concatenating point clouds...", "Abort", 0, nodes_vis.size(), this);
    progress.setWindowModality(Qt::WindowModal);
    int skip = ui.spinSkipPointCloud->value();
    for (int i=0; i<nodes_vis.size(); i+=skip) {
        progress.setValue(i);
        if (progress.wasCanceled()) {
            return;
        }
        if (mCloudFileList[i].empty()) {
            continue;
        }
        Eigen::Isometry3d pose = nodes_vis[i].toIsometry3();
        pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_scan(new pcl::PointCloud<pcl::PointXYZI>);
        // load point cloud file
        pcl::PointCloud<pcl::PointXYZI>::Ptr scan_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        QString cloud_file = mP2oDir.filePath(QString::fromStdString(mCloudFileList[i]));
        if (pcl::io::loadPCDFile(cloud_file.toStdString(), *scan_cloud) == -1) {
            std::cerr << "Failed to load scan cloud file." << std::endl;
            return;
        }

        pcl::transformPointCloud(*scan_cloud, *transformed_scan, pose.matrix());
        *mMapCloud += *transformed_scan;
    }

    if (ui.checkVoxelGridFilter->isChecked()) {
        pcl::VoxelGrid<pcl::PointXYZI> vg;
        vg.setInputCloud(mMapCloud);
        vg.setLeafSize(ui.spinVoxelGridSize->value(), ui.spinVoxelGridSize->value(), ui.spinVoxelGridSize->value());
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
        vg.filter(*cloud_filtered);
        mMapCloud = cloud_filtered;
    }

    // display point cloud map
    mViewer->removePointCloud("map_cloud");
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(mMapCloud, "intensity");
    mViewer->addPointCloud<pcl::PointXYZI>(mMapCloud, intensity_distribution, "map_cloud");
    mViewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "map_cloud");
    mViewer->getRenderWindow()->Render();
}

void MainWindow::onLoadP2oFile() {
    QString p2ofile = QFileDialog::getOpenFileName(this, tr("Open P2O File"), mLastFilePath, tr("P2O File (*.p2o)"));
    if (p2ofile.isEmpty()) {
        return;
    }
    if (loadP2OFile(p2ofile.toLocal8Bit().data(), mNodes, mErrorFuncs, mCloudFileList)) {
        mP2oDir = QFileInfo(p2ofile).absoluteDir();
        resetViewer();

        showPoseGraph(mNodes, mErrorFuncs, mPoseGraphVis);
        mLastFilePath = p2ofile;
    } else {
        QMessageBox::warning(this, "Error", "Failed to load P2O file.");
    }
}

void MainWindow::showPoseGraph(const p2o::Pose3DVec &poses, const std::vector<p2o::ErrorFunc3D*> &errorfuncs, PoseGraphVis &vis) {
    vtkSmartPointer<vtkPoints> vtk_points = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkPolyLine> polyline = vtkSmartPointer<vtkPolyLine>::New();

    Eigen::Vector3f c = vis.color;
    vis.removeObjects(mViewer);
    // Display nodes
    for(int i = 0; i < poses.size(); i++) {
        vtk_points->InsertNextPoint(poses[i].x, poses[i].y, poses[i].z);
        polyline->GetPointIds()->InsertNextId(i);
    }
    vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();
    cells->InsertNextCell(polyline);

    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(vtk_points);
    polyData->SetLines(cells);

    char graph_name[100];
    sprintf(graph_name, "posegraph_%s", vis.name.c_str());
    mViewer->addModelFromPolyData(polyData, graph_name);
    mViewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, c[0], c[1], c[2], graph_name);
    vis.nodes_poly = graph_name;

    // Display node orientation
    if (ui.checkShowNodeOrientation->isChecked()) {
        double axis_size = ui.spinAxisSize->value();
        int skip = ui.spinSkipPose->value();
        for (int i = 0; i < poses.size(); i += skip) {
            Eigen::Affine3f aff = poses[i].toIsometry3().cast<float>();
            char frame_name[100];
            sprintf(frame_name, "axis_%s_%d", vis.name.c_str(), i);
            mViewer->addCoordinateSystem(axis_size, aff, frame_name);
            vis.node_axes.push_back(frame_name);
        }
    }

    // Display edges
    if (ui.checkShowEdges->isChecked()) {
        c *= 0.5;
        for(int i = 0; i < errorfuncs.size(); i++) {
            p2o::ErrorFunc3D *err = errorfuncs[i];
            if (abs(err->ida - err->idb) == 1) continue;
            char edge_name[256];
            sprintf(edge_name, "edge_%s_%d", vis.name.c_str(), i);
            mViewer->addLine(pcl::PointXYZ(poses[err->ida].x, poses[err->ida].y, poses[err->ida].z),
                            pcl::PointXYZ(poses[err->idb].x, poses[err->idb].y, poses[err->idb].z), c[0], c[1], c[2], edge_name);
            vis.edges_lines.push_back(edge_name);
        }
    }

    mViewer->getRenderWindow()->Render();
}

void MainWindow::onLoadPCDFile() {
    QString pcdfile = QFileDialog::getOpenFileName(this, tr("Open PCD File"), "", tr("PCD File (*.pcd)"));
    if (pcdfile.isEmpty()) {
        return;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    if (pcl::io::loadPCDFile(pcdfile.toLocal8Bit().data(), *cloud) == -1) {
        std::cerr << "Failed to load PCD file." << std::endl;
        return;
    }
    mViewer->removePointCloud("map_cloud");
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(cloud, "intensity");
    mViewer->addPointCloud<pcl::PointXYZI>(cloud, intensity_distribution, "map_cloud");
    mViewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "map_cloud");
    mViewer->getRenderWindow()->Render();
}

void MainWindow::onPushHidePointCloudMap() {
    mViewer->removePointCloud("map_cloud");
    mViewer->getRenderWindow()->Render();
}

void MainWindow::onSavePCDFile() {
    if (mMapCloud->empty()) {
        QMessageBox::warning(this, "Error", "No point cloud map loaded.");
        return;
    }
    QString pcdfile = QFileDialog::getSaveFileName(this, tr("Save PCD File"), "", tr("PCD File (*.pcd)"));
    if (pcdfile.isEmpty()) {
        return;
    }
    pcl::io::savePCDFileBinary(pcdfile.toLocal8Bit().data(), *mMapCloud);
}

void MainWindow::loadSettings() {
    QSettings conf;
    mLastFilePath = conf.value("LastP2oFilePath").toString();
}

void MainWindow::saveSettings() {
    QSettings conf;
    conf.setValue("LastP2oFilePath", mLastFilePath);
    conf.sync();
}

void MainWindow::resetViewer() {
    mViewer->removeAllCoordinateSystems();
    mViewer->removeAllShapes();
    mViewer->removeAllPointClouds();
    mViewer->addCoordinateSystem();
}

void MainWindow::onShowNodeOrientationCheckedChanged(int state) {
    bool enabled = state==Qt::Checked;
    ui.spinSkipPose->setEnabled(enabled);
    ui.labelOrientationSkip->setEnabled(enabled);
    ui.spinAxisSize->setEnabled(enabled);
    ui.labelAxisSize->setEnabled(enabled);
}

void MainWindow::onShowEdgesCheckedChanged(int state) {
}

