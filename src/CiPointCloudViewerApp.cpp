#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/Camera.h"
#include "cinder/gl/gl.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <map>

#include "CinderImGui.h"

#include "grabber/PcdGrabber.hpp"

using namespace ci;
using namespace ci::app;
using namespace std;

class CiPointCloudViewerApp : public App {
public:
    void setup() override;
    void mouseDown( MouseEvent event ) override;
    void update() override;
    void draw() override;

private:
    const int kWindowSpacing = 8;
    const int kWindowWidth = 360;

    map<fs::path, shared_ptr<grabber::PointCloudGrabber>> grabbers_;

    gl::VertBatchRef batch_;

    CameraPersp camera_;
    vec3 camera_target_     = vec3(0, 0.5, 0);
    vec3 camera_eye_point_  = vec3(4, 2, -4);

    bool visible_camera_window_     = true;
    bool visible_appearance_window_ = true;
    bool visible_filters_window_    = true;
    bool visible_clouds_window_     = true;
    bool visible_debug_window_      = true;

    int cloud_size_ = 0;
    int filtered_cloud_size_ = 0;

    float point_size_   = 1.0f;
    bool visible_grid_  = true;

    Color bg_color_ = Color(0, 0, 0);

    float voxel_size_           = 0.01f;
    bool enabled_voxel_filter_  = false;

    bool enable_pass_through_x_ = false;
    float min_pass_through_x_   = -1.0f;
    float max_pass_through_x_   = 1.0f;
    bool enable_pass_through_y_ = false;
    float min_pass_through_y_   = -1.0f;
    float max_pass_through_y_   = 1.0f;
    bool enable_pass_through_z_ = false;
    float min_pass_through_z_   = -1.0f;
    float max_pass_through_z_   = 1.0f;

    int sor_meank_              = 50;
    float sor_std_dev_mul_th_   = 1.0f;
    bool enabled_sor_           = false;

    void updatePointCloud();
};

void CiPointCloudViewerApp::setup()
{
    batch_ = gl::VertBatch::create(GL_POINTS);

    ui::initialize();

    gl::enableDepthRead();
    gl::enableDepthWrite();
}

void CiPointCloudViewerApp::updatePointCloud() {
    batch_->clear();

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZRGBA>);

    for (auto grabber_ : grabbers_) {
        *cloud += *(grabber_.second->cloud());
    }

    cloud_size_ = cloud->size();

    if (enable_pass_through_x_) {
        pcl::PassThrough<pcl::PointXYZRGBA> pass_x;
        pass_x.setInputCloud(cloud);
        pass_x.setFilterFieldName("x");
        pass_x.setFilterLimits(min_pass_through_x_, max_pass_through_x_);
        pass_x.filter(*cloud_tmp);
    } else {
        pcl::copyPointCloud(*cloud, *cloud_tmp);
    }

    if (enable_pass_through_y_) {
        pcl::PassThrough<pcl::PointXYZRGBA> pass_y;
        pass_y.setInputCloud(cloud_tmp);
        pass_y.setFilterFieldName("y");
        pass_y.setFilterLimits(min_pass_through_y_, max_pass_through_y_);
        pass_y.filter(*cloud);
    } else {
        pcl::copyPointCloud(*cloud_tmp, *cloud);
    }

    if (enable_pass_through_z_) {
        pcl::PassThrough<pcl::PointXYZRGBA> pass_z;
        pass_z.setInputCloud(cloud);
        pass_z.setFilterFieldName("z");
        pass_z.setFilterLimits(min_pass_through_z_, max_pass_through_z_);
        pass_z.filter(*cloud_tmp);
    } else {
        pcl::copyPointCloud(*cloud, *cloud_tmp);
    }

    if (enabled_voxel_filter_) {
        pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
        sor.setInputCloud(cloud_tmp);
        sor.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
        sor.filter(*cloud);
    } else {
        pcl::copyPointCloud(*cloud_tmp, *cloud);
    }

    if (enabled_sor_) {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor;
        sor.setInputCloud(cloud);
        sor.setMeanK(sor_meank_);
        sor.setStddevMulThresh(sor_std_dev_mul_th_);
        sor.filter(*cloud_tmp);
        pcl::copyPointCloud(*cloud_tmp, *cloud);
    }

    for (auto point : cloud->points) {
        batch_->color(ci::ColorA8u(point.r, point.g, point.b, point.a));
        batch_->vertex(ci::vec3(point.x, point.y, point.z));
    }

    filtered_cloud_size_ = cloud->size();
}

void CiPointCloudViewerApp::mouseDown( MouseEvent event )
{
}

void CiPointCloudViewerApp::update()
{
    auto windowPos = vec2(kWindowSpacing, kWindowSpacing);
    bool updated = false;
    {
        ui::ScopedMainMenuBar menuBar;

        if (ui::BeginMenu("File")) {
            if (ui::MenuItem("Open *.pcd file")) {
                auto pcdfile = getOpenFilePath();
                auto grabber = std::make_shared<grabber::PcdGrabber>(pcdfile);
                grabbers_[pcdfile] = grabber;
                grabber->start([this]() {
                    updatePointCloud();
                });
            }
            ui::EndMenu();
        }

        if (ui::BeginMenu("View")) {
            ui::MenuItem("Show grid", nullptr, &visible_grid_);
            ui::EndMenu();
        }

        if (ui::BeginMenu("Window")) {
            ui::MenuItem("Camera",      nullptr, &visible_camera_window_);
            ui::MenuItem("Appearance",  nullptr, &visible_appearance_window_);
            ui::MenuItem("Filter",      nullptr, &visible_filters_window_);
            ui::MenuItem("Clouds",      nullptr, &visible_clouds_window_);
            ui::MenuItem("Debug",       nullptr, &visible_debug_window_);
            ui::EndMenu();
        }

        windowPos.y += ui::GetItemRectSize().y;
        ui::SetNextWindowPos(windowPos);
        ui::SetNextWindowSize(vec2(kWindowWidth, 0));
    }
    if (visible_camera_window_) {
        ui::ScopedWindow window("Camera");
        ui::DragFloat3("Look at", &camera_target_[0]);
        ui::DragFloat3("Eye point", &camera_eye_point_[0]);

        windowPos.y += ui::GetWindowHeight() + kWindowSpacing;
        ui::SetNextWindowPos(windowPos);
        ui::SetNextWindowSize(vec2(kWindowWidth, 0));
    }
    if (visible_appearance_window_) {
        ui::ScopedWindow window("Appearance");
        ui::InputFloat("Point size", &point_size_, 0.1f);
        ui::ColorEdit3("Background color", &bg_color_[0]);
        windowPos.y += ui::GetWindowHeight() + kWindowSpacing;
        ui::SetNextWindowPos(windowPos);
        ui::SetNextWindowSize(vec2(kWindowWidth, 0));
    }
    if (visible_filters_window_) {
        ui::ScopedWindow window("Filters");
        int id = 0;
        auto addFilter = [&](const char *label, std::function<void()> definition) {
            if (ui::TreeNode(label)) {
                ui::Columns(2);
                definition();
                ui::TreePop();
                ui::Columns(1);
            }
        };
        auto addUi = [&](const char *label, std::function<bool()> updater) {
            ui::PushID(id++);                           
            ui::AlignFirstTextHeightToWidgets();
            ui::Bullet();
            ui::TextUnformatted(label);
            ui::NextColumn();
            ui::PushItemWidth(-1);
            updated = updated || updater();
            ui::PopItemWidth();
            ui::NextColumn();
            ui::PopID();
        };
        addFilter("Pass-through X", [&](){
            addUi("Enable", [&](){
                return ui::Checkbox("##value", &enable_pass_through_x_);
            });
            addUi("Min X", [&](){
                return ui::SliderFloat("##value", &min_pass_through_x_, -10, max_pass_through_x_);
            });
            addUi("Max X", [&](){
                return ui::SliderFloat("##value", &max_pass_through_x_, min_pass_through_x_, 10);
            });
        });
        addFilter("Pass-through Y", [&](){
            addUi("Enable", [&](){
                return ui::Checkbox("##value", &enable_pass_through_y_);
            });
            addUi("Min Y", [&](){
                return ui::SliderFloat("##value", &min_pass_through_y_, -10, max_pass_through_y_);
            });
            addUi("Max Y", [&](){
                return ui::SliderFloat("##value", &max_pass_through_y_, min_pass_through_y_, 10);
            });
        });
        addFilter("Pass-through Z", [&](){
            addUi("Enable", [&](){
                return ui::Checkbox("##value", &enable_pass_through_z_);
            });
            addUi("Min Z", [&](){
                return ui::SliderFloat("##value", &min_pass_through_z_, -10, max_pass_through_z_);
            });
            addUi("Max Z", [&](){
                return ui::SliderFloat("##value", &max_pass_through_z_, min_pass_through_z_, 10);
            });
        });
        addFilter("Voxel filter", [&](){
            addUi("Enable", [&](){
                return ui::Checkbox("##value", &enabled_voxel_filter_);
            });
            addUi("Voxel size", [&](){
                return ui::InputFloat("##value", &voxel_size_, 0.0001f);
            });
        });
        addFilter("Statistical outlier removal", [&](){
            addUi("Enable", [&](){
                return ui::Checkbox("##value", &enabled_sor_);
            });
            addUi("MeanK", [&](){
                return ui::InputInt("##value", &sor_meank_, 1);
            });
            addUi("StddevMulThresh", [&](){
                return ui::InputFloat("##value", &sor_std_dev_mul_th_, 0.1f);
            });
        });

        windowPos.y += ui::GetWindowHeight() + kWindowSpacing;
        ui::SetNextWindowPos(windowPos);
        ui::SetNextWindowSize(vec2(kWindowWidth, 0));
    }
    if (visible_clouds_window_) {
        ui::ScopedWindow window("Clouds");
        ui::ListBoxHeader("");
        for (auto pair: grabbers_) {
            ui::Selectable(pair.first.filename().c_str());
        }
        ui::ListBoxFooter();

        windowPos.y += ui::GetWindowHeight() + kWindowSpacing;
        ui::SetNextWindowPos(windowPos);
        ui::SetNextWindowSize(vec2(kWindowWidth, 0));
    }
    if (visible_debug_window_) {
        ui::ScopedWindow window("Debug");
        ui::LabelText("FPS", "%f", getAverageFps());
        ui::LabelText("Cloud size", "%d", cloud_size_);
        ui::LabelText("Filtered", "%d", filtered_cloud_size_);
    }

    if (updated) {
        updatePointCloud();
    }
}

void CiPointCloudViewerApp::draw()
{
    gl::clear(bg_color_);

    camera_.lookAt(camera_eye_point_, camera_target_);
    gl::setMatrices(camera_);

    gl::pointSize(point_size_);

    if (visible_grid_) {
        gl::pushMatrices();
        gl::color(1, 1, 1, 0.3);
        gl::rotate(M_PI_2, vec3(1, 0, 0));
        for (float i = -5; i < 5; i += 0.5) {
            for (float j = -5; j < 5; j += 0.5) {
                gl::drawStrokedRect(Rectf(i, j, i + 0.5, j + 0.5));
            }
        }
        gl::popMatrices();
    }

    batch_->draw();
}

CINDER_APP( CiPointCloudViewerApp, RendererGl, [](App::Settings *settings) {
    settings->setHighDensityDisplayEnabled();
    settings->setWindowSize(1280, 960);
    settings->setFrameRate(240.0f);
})

