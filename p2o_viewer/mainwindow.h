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
    void showPoseGraph(const p2o::Pose3DVec &poses, const std::vector<p2o::ErrorFunc3D*> &errorfuncs, const std::string &name);

    Ui::MainWindow ui;
    QStatusBar mStatusBar;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> mViewer;
    p2o::Pose3DVec mNodes;
    p2o::Pose3DVec mResultNodes;
    QDir mP2oDir;
    std::vector<p2o::ErrorFunc3D*> mErrorFuncs;
    std::vector<std::string> mCloudFileList;
    pcl::PointCloud<pcl::PointXYZI>::Ptr mMapCloud;
    QString mLastFilePath;
};

#endif // MAINWINDOW_H

