#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/Camera.h"
#include "cinder/CameraUi.h"
#include "cinder/gl/gl.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <set>
#include <map>

#include "CinderImGui.h"

#include "grabber/PcdGrabber.hpp"

using namespace ci;
using namespace ci::app;
using namespace std;

class CiPointCloudViewerApp : public App {
public:
    void setup() override;
    void mouseDown(MouseEvent event) override;
    void mouseDrag(MouseEvent event) override;
    void mouseWheel(MouseEvent event) override;
    void update() override;
    void draw() override;

private:
    const ImGuiWindowFlags kWindowFlags = ImGuiWindowFlags_ShowBorders;
    const int kWindowSpacing = 8;
    const int kWindowWidth = 360;

    const ColorA8u kColorBlackA55   = ColorA8u(0x22, 0x22, 0x22, 0x55);
    const ColorA8u kColorBlackAcc   = ColorA8u(0x22, 0x22, 0x22, 0xcc);
    // bleu de provence
    const ColorA8u kColorPrimary    = ColorA8u(0x00, 0x9a, 0xc5, 0xcc);
    const ColorA8u kColorPrimaryA99 = ColorA8u(0x00, 0x9a, 0xc5, 0x99);
    const ColorA8u kColorPrimaryA22 = ColorA8u(0x00, 0x9a, 0xc5, 0x22);
    // rosso di toscana
    const ColorA8u kColorAccent     = ColorA8u(0xf1, 0x67, 0x3f, 0xee);
    const ColorA8u kColorAccentAcc  = ColorA8u(0xf1, 0x67, 0x3f, 0xcc);
    const ColorA8u kColorAccentA99  = ColorA8u(0xf1, 0x67, 0x3f, 0x99);

    map<fs::path, shared_ptr<grabber::PointCloudGrabber>> grabbers_;
    shared_ptr<grabber::PointCloudGrabber> grabber_selected_;
    set<fs::path> hidden_clouds_;

    gl::VertBatchRef batch_;

    CameraPersp camera_;
    CameraUi camera_ui_;
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

    Color bg_color_ = Color8u(0x11, 0x11, 0x11);

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
    camera_ui_ = CameraUi(&camera_);

    camera_.setEyePoint(camera_eye_point_);
    camera_.lookAt(camera_eye_point_, camera_target_);

    auto options = ui::Options()
        .darkTheme()
        .color(ImGuiCol_MenuBarBg,              kColorPrimaryA22)
        .color(ImGuiCol_TitleBg,                kColorPrimaryA22)
        .color(ImGuiCol_TitleBgCollapsed,       kColorPrimaryA22)
        .color(ImGuiCol_TitleBgActive,          kColorPrimaryA99)
        .color(ImGuiCol_WindowBg,               kColorPrimaryA22)
        .color(ImGuiCol_Border,                 kColorPrimaryA99)
        .color(ImGuiCol_FrameBg,                kColorPrimaryA22)
        .color(ImGuiCol_FrameBgHovered,         kColorAccentAcc)
        .color(ImGuiCol_FrameBgActive,          kColorAccent)
        .color(ImGuiCol_ScrollbarBg,            kColorPrimaryA22)
        .color(ImGuiCol_ScrollbarGrab,          kColorPrimaryA99)
        .color(ImGuiCol_ScrollbarGrabHovered,   kColorPrimaryA99)
        .color(ImGuiCol_ScrollbarGrabActive,    kColorPrimary)
        .color(ImGuiCol_CheckMark,              kColorAccent)
        .color(ImGuiCol_SliderGrab,             kColorPrimaryA99)
        .color(ImGuiCol_SliderGrabActive,       kColorPrimary)
        .color(ImGuiCol_Button,                 kColorPrimaryA22)
        .color(ImGuiCol_ButtonHovered,          kColorAccentAcc)
        .color(ImGuiCol_ButtonActive,           kColorAccent)
        .color(ImGuiCol_Header,                 kColorAccentA99)
        .color(ImGuiCol_HeaderHovered,          kColorAccentAcc)
        .color(ImGuiCol_HeaderActive,           kColorAccent)
        .color(ImGuiCol_Column,                 kColorBlackA55)
        .color(ImGuiCol_ColumnHovered,          kColorAccentAcc)
        .color(ImGuiCol_ColumnActive,           kColorAccent)
        .color(ImGuiCol_Text,                   kColorPrimary)
        .color(ImGuiCol_TextDisabled,           kColorBlackA55)
        .color(ImGuiCol_TextSelectedBg,         kColorAccent)
        .color(ImGuiCol_PopupBg,                kColorBlackAcc)
        .antiAliasedLines(true)
        .antiAliasedShapes(true)
        .windowRounding(0.0f)
        .frameRounding(0.0f);
    ui::initialize(options);

    gl::enableDepthRead();
    gl::enableDepthWrite();
}

void CiPointCloudViewerApp::updatePointCloud() {
    batch_->clear();

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZRGBA>);

    for (auto grabber : grabbers_) {
        if (hidden_clouds_.find(grabber.first) == hidden_clouds_.end()) {
            *cloud += *(grabber.second->cloud());
        }
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

void CiPointCloudViewerApp::mouseDown(MouseEvent event) {
    camera_ui_.mouseDown(event);
}

void CiPointCloudViewerApp::mouseDrag(MouseEvent event) {
    camera_ui_.mouseDrag(event);
}

void CiPointCloudViewerApp::mouseWheel(MouseEvent event) {
    camera_ui_.mouseWheel(event);
}

void CiPointCloudViewerApp::update()
{
    auto windowPos = vec2(kWindowSpacing, kWindowSpacing);
    bool updated = false;
    camera_eye_point_ = camera_.getEyePoint();
    camera_target_ = camera_.getPivotPoint();
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
        ui::ScopedWindow window("Camera", kWindowFlags);
        if (ui::DragFloat3("Look at", &camera_target_[0])) {
            camera_.lookAt(camera_eye_point_, camera_target_);
        }
        if (ui::DragFloat3("Eye point", &camera_eye_point_[0])) {
            camera_.setEyePoint(camera_eye_point_);
        }

        windowPos.y += ui::GetWindowHeight() + kWindowSpacing;
        ui::SetNextWindowPos(windowPos);
        ui::SetNextWindowSize(vec2(kWindowWidth, 0));
    }
    if (visible_appearance_window_) {
        ui::ScopedWindow window("Appearance", kWindowFlags);
        ui::InputFloat("Point size", &point_size_, 0.1f);
        ui::ColorEdit3("Background color", &bg_color_[0]);
        windowPos.y += ui::GetWindowHeight() + kWindowSpacing;
        ui::SetNextWindowPos(windowPos);
        ui::SetNextWindowSize(vec2(kWindowWidth, 0));
    }
    if (visible_filters_window_) {
        ui::ScopedWindow window("Filters", kWindowFlags);
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
        ui::ScopedWindow window("Clouds", kWindowFlags);

        if (ui::Button("Clear")) {
            grabbers_.clear();
            grabber_selected_ = nullptr;
            updated = true;
        }

        if (grabber_selected_) {
            ui::SameLine();
            if (ui::Button("Remove")) {
                grabbers_.erase(grabber_selected_->path());
                grabber_selected_ = nullptr;
                updated = true;
            }

            ui::SameLine();
            if (hidden_clouds_.find(grabber_selected_->path()) != hidden_clouds_.end()) {
                if (ui::Button("Show")) {
                    hidden_clouds_.erase(grabber_selected_->path());
                    updated = true;
                }
            } else {
                if (ui::Button("Hide")) {
                    hidden_clouds_.insert(grabber_selected_->path());
                    updated = true;
                }
            }
        }

        ui::ListBoxHeader("");
        for (auto pair: grabbers_) {
            if (ui::Selectable(pair.first.filename().c_str(), grabber_selected_ && (grabber_selected_->path() == pair.first))) {
                grabber_selected_ = pair.second;
            }
        }
        ui::ListBoxFooter();

        windowPos.y += ui::GetWindowHeight() + kWindowSpacing;
        ui::SetNextWindowPos(windowPos);
        ui::SetNextWindowSize(vec2(kWindowWidth, 0));
    }
    if (visible_debug_window_) {
        ui::ScopedWindow window("Information", kWindowFlags);
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

