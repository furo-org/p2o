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
    QMainWindow(parent) {
    loadSettings();
    ui.setupUi(this);
    connectSignalsAndSlots();
    auto renderer = vtkSmartPointer<vtkRenderer>::New();
    auto renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    renderWindow->AddRenderer(renderer);
    viewer.reset (new pcl::visualization::PCLVisualizer(renderer, renderWindow, "PCL Viewer", false));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCoordinateSystem(1.0);
    ui.qvtkWidget->setRenderWindow(viewer->getRenderWindow());
    this->setStatusBar(&statusBar);
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
}

void MainWindow::onPushOptimizePoseGraph() {
    p2o::Optimizer3D opt;
    int min_steps = ui.spinMinSteps->value();
    int max_steps = ui.spinMaxSteps->value();
    nodes_result = opt.optimizePath(nodes, errorfuncs, max_steps, min_steps);
    showPoseGraph(nodes_result, errorfuncs, "graph_out");
}

void MainWindow::onPushShowPointCloudMap() {
    p2o::Pose3DVec &nodes_vis = this->nodes_result;
    if (nodes_vis.empty()) {
        nodes_vis = nodes;
    }
    if (nodes_vis.empty()) {
        QMessageBox::warning(this, "Error", "No pose graph loaded.");
        return;
    }
    map_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);

    QProgressDialog progress("Concatenating point clouds...", "Abort", 0, nodes_vis.size(), this);
    progress.setWindowModality(Qt::WindowModal);
    int skip = ui.spinSkipPointCloud->value();
    for (int i=0; i<nodes_vis.size(); i+=skip) {
        progress.setValue(i);
        if (progress.wasCanceled()) {
            return;
        }
        if (cloud_files[i].empty()) {
            continue;
        }
        Eigen::Isometry3d pose = nodes_vis[i].toIsometry3();
        pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_scan(new pcl::PointCloud<pcl::PointXYZI>);
        // load point cloud file
        pcl::PointCloud<pcl::PointXYZI>::Ptr scan_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        QString cloud_file = p2o_dir.filePath(QString::fromStdString(cloud_files[i]));
        if (pcl::io::loadPCDFile(cloud_file.toStdString(), *scan_cloud) == -1) {
            std::cerr << "Failed to load scan cloud file." << std::endl;
            return;
        }

        pcl::transformPointCloud(*scan_cloud, *transformed_scan, pose.matrix());
        *map_cloud += *transformed_scan;
    }

    if (ui.checkVoxelGridFilter->isChecked()) {
        pcl::VoxelGrid<pcl::PointXYZI> vg;
        vg.setInputCloud(map_cloud);
        vg.setLeafSize(ui.spinVoxelGridSize->value(), ui.spinVoxelGridSize->value(), ui.spinVoxelGridSize->value());
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
        vg.filter(*cloud_filtered);
        map_cloud = cloud_filtered;
    }

    // display point cloud map
    viewer->removePointCloud("map_cloud");
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(map_cloud, "intensity");
    viewer->addPointCloud<pcl::PointXYZI>(map_cloud, intensity_distribution, "map_cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "map_cloud");
    viewer->getRenderWindow()->Render();
}

void MainWindow::onLoadP2oFile() {
    QString p2ofile = QFileDialog::getOpenFileName(this, tr("Open P2O File"), mLastFilePath, tr("P2O File (*.p2o)"));
    if (p2ofile.isEmpty()) {
        return;
    }
    if (loadP2OFile(p2ofile.toLocal8Bit().data(), nodes, errorfuncs, cloud_files)) {
        p2o_dir = QFileInfo(p2ofile).absoluteDir();
        viewer->removeAllShapes();
        viewer->removeAllPointClouds();
        showPoseGraph(nodes, errorfuncs, "graph_in");
        mLastFilePath = p2ofile;
    } else {
        QMessageBox::warning(this, "Error", "Failed to load P2O file.");
    }
}

void MainWindow::showPoseGraph(const p2o::Pose3DVec &poses, const std::vector<p2o::ErrorFunc3D*> &errorfuncs, const std::string &name) {
    vtkSmartPointer<vtkPoints> vtk_points = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkPolyLine> polyline = vtkSmartPointer<vtkPolyLine>::New();

    Eigen::Vector3f c;
    if (name == "graph_in") {
        c << 1.0, 1.0, 1.0;
    } else {
        c << 1.0, 0.0, 0.0;
    }
    for(int i = 0; i < poses.size(); i++) {
        vtk_points->InsertNextPoint(poses[i].x, poses[i].y, poses[i].z);
        polyline->GetPointIds()->InsertNextId(i);
    }
    for(int i = 0; i < errorfuncs.size(); i++) {
        p2o::ErrorFunc3D *err = errorfuncs[i];
        //viewer->addLine(pcl::PointXYZ(poses[err->ida].x, poses[err->ida].y, poses[err->ida].z),
        //                pcl::PointXYZ(poses[err->idb].x, poses[err->idb].y, poses[err->idb].z), c[0], c[1], c[2], edge_name);
    }

    vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();
    cells->InsertNextCell(polyline);

    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(vtk_points);
    polyData->SetLines(cells);

    char graph_name[100];
    sprintf(graph_name, "posegraph_%s", name.c_str());
    viewer->addModelFromPolyData(polyData, graph_name);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, c[0], c[1], c[2], graph_name);

    if (ui.checkCoordinateAxis->isChecked()) {
        int skip = ui.spinSkipPose->value();
        for (int i = 0; i < poses.size(); i += skip) {
            Eigen::Affine3f aff = poses[i].toIsometry3().cast<float>();
            char frame_name[100];
            sprintf(frame_name, "axis_%s_%d", name.c_str(), i);
            viewer->addCoordinateSystem(0.5, aff, frame_name);
        }
    }
    viewer->getRenderWindow()->Render();
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
    viewer->removePointCloud("map_cloud");
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(cloud, "intensity");
    viewer->addPointCloud<pcl::PointXYZI>(cloud, intensity_distribution, "map_cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "map_cloud");
    viewer->getRenderWindow()->Render();
}

void MainWindow::onPushHidePointCloudMap() {
    viewer->removePointCloud("map_cloud");
    viewer->getRenderWindow()->Render();
}

void MainWindow::onSavePCDFile() {
    if (map_cloud->empty()) {
        QMessageBox::warning(this, "Error", "No point cloud map loaded.");
        return;
    }
    QString pcdfile = QFileDialog::getSaveFileName(this, tr("Save PCD File"), "", tr("PCD File (*.pcd)"));
    if (pcdfile.isEmpty()) {
        return;
    }
    pcl::io::savePCDFileBinary(pcdfile.toLocal8Bit().data(), *map_cloud);
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

