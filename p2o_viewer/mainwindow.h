#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QStatusBar>
#include <QDir>
#include "p2o.h"
#include <pcl/visualization/pcl_visualizer.h>
#include "ui_mainwindow.h"

class MainWindow : public QMainWindow {
    Q_OBJECT

public slots:
    void onLoadP2oFile();
    void onLoadPCDFile();
    void onSavePCDFile();
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
    void loadP2oFile(const std::string &filename);
    void showPoseGraph(const p2o::Pose3DVec &poses, const std::vector<p2o::ErrorFunc3D*> &errorfuncs, const std::string &name);

    Ui::MainWindow ui;
    QStatusBar statusBar;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    p2o::Pose3DVec nodes;
    p2o::Pose3DVec nodes_result;
    QDir p2o_dir;
    std::vector<p2o::ErrorFunc3D*> errorfuncs;
    std::vector<std::string> cloud_files;
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud;
    QString mLastFilePath;
};

#endif // MAINWINDOW_H

