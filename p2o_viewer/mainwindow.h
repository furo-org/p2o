/****************************************************************************
 * p2o: Petite Portable Pose-graph Optimizer v2
 * Copyright (C) 202024 Kiyoshi Irie
 * Copyright (C) 2024 Future Robotics Technology Center (fuRo),
 *                    Chiba Institute of Technology.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at https://mozilla.org/MPL/2.0/.
 ****************************************************************************/
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QStatusBar>
#include <QDir>
#include "p2o.h"
#include <pcl/visualization/pcl_visualizer.h>
#include "ui_mainwindow.h"

struct PoseGraphVis {
    std::string name;
    Eigen::Vector3f color = Eigen::Vector3f(1, 1, 1);
    std::string nodes_poly;
    std::vector<std::string> node_axes;
    std::vector<std::string> edges_lines;
    void removeObjects(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer) {
        viewer->removeShape(nodes_poly);
        for(auto &node : node_axes) {
            viewer->removeCoordinateSystem(node);
        }
        for(auto &edge : edges_lines) {
            viewer->removeShape(edge);
        }
        nodes_poly = "";
        node_axes.clear();
        edges_lines.clear();
    }
    PoseGraphVis(const std::string &name, const Eigen::Vector3f &color) : name(name), color(color) {}
};

class MainWindow : public QMainWindow {
    Q_OBJECT

public slots:
    void onLoadP2oFile();
    void onLoadPCDFile();
    void onSavePCDFile();
    void onShowNodeOrientationCheckedChanged(int state);
    void onShowEdgesCheckedChanged(int state);
    void onPushOptimizePoseGraph();
    void onPushShowPointCloudMap();
    void onPushHidePointCloudMap();

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
    void connectSignalsAndSlots();
    void loadSettings();
    void saveSettings();
    void resetViewer();
    void showPoseGraph(const p2o::Pose3DVec &poses, const std::vector<p2o::ErrorFunc3D*> &errorfuncs, PoseGraphVis &vis);

    Ui::MainWindow ui;
    QStatusBar mStatusBar;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> mViewer;
    p2o::Pose3DVec mNodes;
    PoseGraphVis mPoseGraphVis;
    p2o::Pose3DVec mResultNodes;
    PoseGraphVis mResultPoseGraphVis;
    QDir mP2oDir;
    std::vector<p2o::ErrorFunc3D*> mErrorFuncs;
    std::vector<std::string> mCloudFileList;
    pcl::PointCloud<pcl::PointXYZI>::Ptr mMapCloud;
    QString mLastFilePath;
};

#endif // MAINWINDOW_H

