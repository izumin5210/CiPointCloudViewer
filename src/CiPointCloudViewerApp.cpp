#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/Camera.h"
#include "cinder/CameraUi.h"
#include "cinder/gl/gl.h"
#include "cinder/Signals.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <set>
#include <map>

#include "CinderImGui.h"

#include "grabber/PcdGrabber.hpp"
#include "grabber/SequentialPcdGrabber.hpp"

#include "filter/PassThroughFilter.hpp"
#include "filter/VoxelFilter.hpp"
#include "filter/StatisticalOutlierRemovalFilter.hpp"

#include "model/CloudsManager.h"
#include "io/CalibrationParamsManager.h"
#include "io/SensorDeviceManager.hpp"

using namespace ci;
using namespace ci::app;
using namespace std;

namespace bfs = boost::filesystem;
typedef bfs::path bpath;
typedef signals::Signal<void (const gl::VertBatchRef&)> EventSignalBatch;

class CiPointCloudViewerApp : public App {
public:
    CiPointCloudViewerApp();
    ~CiPointCloudViewerApp();

    void setup() override;
    void mouseDown(MouseEvent event) override;
    void mouseDrag(MouseEvent event) override;
    void mouseWheel(MouseEvent event) override;
    void update() override;
    void draw() override;

private:
    const ImGuiWindowFlags kWindowFlags = ImGuiWindowFlags_ShowBorders;
    const int kWindowSpacing = 8;
    const int kWindowWidth = 320;
    const int kPlayerWindowHeight = 48;

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
    gl::VertBatchRef grid_batch_;

    CameraPersp camera_;
    CameraUi camera_ui_;
    vec3 camera_target_     = vec3(0, 0.5, 0);
    vec3 camera_eye_point_  = vec3(4, 2, -4);

    bool visible_camera_window_     = true;
    bool visible_appearance_window_ = true;
    bool visible_filters_window_    = true;
    bool visible_clouds_window_     = true;
    bool visible_debug_window_      = true;
    bool visible_player_window_     = true;

    int cloud_size_ = 0;
    int filtered_cloud_size_ = 0;

    float point_size_   = 1.0f;
    bool visible_grid_  = true;

    Color bg_color_ = Color8u(0x11, 0x11, 0x11);

    std::atomic<bool> updated_;

    function<void()> on_cloud_updated_ = [this]() { updated_ = true; };

    map<fs::path, vec2> loading_progresses_;
    function<void(fs::path, int, int)> on_pcd_loaded_ =
        [this](fs::path path, int count, int max) {
            loading_progresses_[path] = vec2(count, max);
        };

    filter::PassThroughFilter<pcl::PointXYZRGBA> x_pass_through_filter_;
    filter::PassThroughFilter<pcl::PointXYZRGBA> y_pass_through_filter_;
    filter::PassThroughFilter<pcl::PointXYZRGBA> z_pass_through_filter_;
    filter::VoxelFilter<pcl::PointXYZRGBA> voxel_filter_;
    filter::StatisticalOutlierRemovalFilter<pcl::PointXYZRGBA> sor_filter_;

    io::SensorDeviceManager sensor_device_manager_;

    map<std::string, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> clouds_;

    mutex vert_batch_mutex_;
    EventSignalBatch signal_batch_updated_;

    void onCloudUpdated(const models::CloudEvent& event);
    void onBatchUpdated(const gl::VertBatchRef& batch);
    void updatePointCloud();
};

CiPointCloudViewerApp::CiPointCloudViewerApp()
    : x_pass_through_filter_("x")
    , y_pass_through_filter_("y")
    , z_pass_through_filter_("z")
    , voxel_filter_()
    , sor_filter_()
    , sensor_device_manager_()
{}

CiPointCloudViewerApp::~CiPointCloudViewerApp() {
    sensor_device_manager_.stop();
}

void CiPointCloudViewerApp::setup()
{
    sensor_device_manager_.start();
    models::CloudsManager::getSignalCloudUpdated()
        .connect(signals::slot(this, &CiPointCloudViewerApp::onCloudUpdated));
    signal_batch_updated_
        .connect(signals::slot(this, &CiPointCloudViewerApp::onBatchUpdated));

    batch_ = gl::VertBatch::create(GL_POINTS);
    grid_batch_ = gl::VertBatch::create(GL_LINES);
    camera_ui_ = CameraUi(&camera_);

    grid_batch_->color(1, 1, 1, 0.3);
    for (float i = -5; i <= 5.0; i += 0.5) {
        for (float j = -5; j <= 5.0; j += 0.5) {
            grid_batch_->vertex(vec3( i, 0, -j));
            grid_batch_->vertex(vec3( i, 0,  j));
            grid_batch_->vertex(vec3(-i, 0,  j));
            grid_batch_->vertex(vec3( i, 0,  j));
        }
    }

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
        .color(ImGuiCol_PlotLines,              kColorPrimaryA99)
        .color(ImGuiCol_PlotLinesHovered,       kColorPrimary)
        .color(ImGuiCol_PlotHistogram,          kColorPrimaryA99)
        .color(ImGuiCol_PlotHistogramHovered,   kColorPrimary)
        .color(ImGuiCol_Text,                   kColorPrimary)
        .color(ImGuiCol_TextDisabled,           kColorBlackA55)
        .color(ImGuiCol_TextSelectedBg,         kColorAccent)
        .color(ImGuiCol_PopupBg,                kColorBlackAcc)
        .antiAliasedLines(true)
        .antiAliasedShapes(true)
        .windowRounding(0.0f)
        .frameRounding(0.0f);
    ui::initialize(options);

    gl::enableFaceCulling(true);
    gl::enableVerticalSync(false);
    disableFrameRate();
    gl::enableDepthRead();
    gl::enableDepthWrite();
}

void CiPointCloudViewerApp::onCloudUpdated(const models::CloudEvent& event) {
    clouds_[event.key] = event.cloud;
    updatePointCloud();
}

void CiPointCloudViewerApp::onBatchUpdated(const gl::VertBatchRef& batch) {
    lock_guard<mutex> lg(vert_batch_mutex_);
    batch_ = batch;
}

void CiPointCloudViewerApp::updatePointCloud() {
    updated_ = false;
    const gl::VertBatchRef batch = gl::VertBatch::create(GL_POINTS);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

    /*
     * for (auto grabber : grabbers_) {
     *     if (hidden_clouds_.find(grabber.first) == hidden_clouds_.end()) {
     *         *cloud += *(grabber.second->cloud());
     *     }
     * }
     */

    for (auto pair : clouds_) {
        *cloud += *(pair.second);
    }

    cloud_size_ = cloud->size();

    if (x_pass_through_filter_.params_.enable) {
        x_pass_through_filter_.filter(cloud);
    }

    if (y_pass_through_filter_.params_.enable) {
        y_pass_through_filter_.filter(cloud);
    }

    if (z_pass_through_filter_.params_.enable) {
        z_pass_through_filter_.filter(cloud);
    }

    if (voxel_filter_.params_.enable) {
        voxel_filter_.filter(cloud);
    }

    if (sor_filter_.params_.enable) {
        sor_filter_.filter(cloud);
    }

    for (auto point : cloud->points) {
        batch->color(ci::ColorA8u(point.r, point.g, point.b, point.a));
        batch->vertex(ci::vec3(point.x, point.y, point.z));
    }

    filtered_cloud_size_ = cloud->size();

    signal_batch_updated_.emit(batch);
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
    auto leftWindowPos = vec2(kWindowSpacing, kWindowSpacing);
    auto rightWindowPos = vec2(getWindowWidth() - (kWindowWidth + kWindowSpacing), kWindowSpacing);
    camera_eye_point_ = camera_.getEyePoint();
    camera_target_ = camera_.getPivotPoint();
    {
        ui::ScopedMainMenuBar menuBar;

        if (ui::BeginMenu("File")) {
            if (ui::MenuItem("Open *.pcd file")) {
                auto pcdfile = getOpenFilePath(bfs::path(), {"pcd"});
                if (bfs::exists(pcdfile)) {
                    auto grabber = std::make_shared<grabber::PcdGrabber>(pcdfile);
                    grabbers_[pcdfile] = grabber;
                    grabber->start(on_cloud_updated_);
                }
            }
            if(ui::MenuItem("Open directory")) {
                auto dir = getFolderPath();
                if (bfs::is_directory(dir)) {
                    auto grabber = std::make_shared<grabber::SequentialPcdGrabber>(dir);
                    grabbers_[dir] = grabber;
                    grabber->initialize(on_pcd_loaded_);
                    // TODO: startボタンみたいなのつくって，そこに移動
//                    grabber->start([this]() {
//                        // TODO: not yet implemented
//                    });
                }
            }
            if (ui::MenuItem("Open calibration yaml file")) {
                auto yamlfile = getOpenFilePath(bfs::path(), {"yaml", "yml"});
                if (bfs::exists(yamlfile)) {
                  io::CalibrationParamsManager::load(yamlfile.string());
                }
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
            ui::MenuItem("Player",      nullptr, &visible_player_window_);
            ui::EndMenu();
        }

        leftWindowPos.y += ui::GetItemRectSize().y;
        rightWindowPos.y += ui::GetItemRectSize().y;
    }
    if (visible_camera_window_) {
        ui::ScopedWindow window("Camera", kWindowFlags);
        if (ui::DragFloat3("Look at", &camera_target_[0])) {
            camera_.lookAt(camera_eye_point_, camera_target_);
        }
        if (ui::DragFloat3("Eye point", &camera_eye_point_[0])) {
            camera_.setEyePoint(camera_eye_point_);
        }

        ui::SetWindowPos(leftWindowPos);
        ui::SetWindowSize(vec2(kWindowWidth, 0));
        leftWindowPos.y += ui::GetWindowHeight() + kWindowSpacing;
    }
    if (visible_appearance_window_) {
        ui::ScopedWindow window("Appearance", kWindowFlags);
        ui::InputFloat("Point size", &point_size_, 0.1f);
        ui::ColorEdit3("Background", &bg_color_[0]);
        ui::SetWindowPos(leftWindowPos);
        ui::SetWindowSize(vec2(kWindowWidth, 0));
        leftWindowPos.y += ui::GetWindowHeight() + kWindowSpacing;
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
            updated_ = updated_ || updater();
            ui::PopItemWidth();
            ui::NextColumn();
            ui::PopID();
        };
        addFilter("Pass-through X", [&](){
            auto params = &x_pass_through_filter_.params_;
            addUi("Enable", [&](){
                return ui::Checkbox("##value", &params->enable);
            });
            addUi("Range", [&](){
                return ui::DragFloatRange2("##value", &params->min, &params->max, 0.05f);
            });
        });
        addFilter("Pass-through Y", [&](){
            auto params = &y_pass_through_filter_.params_;
            addUi("Enable", [&](){
                return ui::Checkbox("##value", &params->enable);
            });
            addUi("Range", [&](){
                return ui::DragFloatRange2("##value", &params->min, &params->max, 0.05f);
            });
        });
        addFilter("Pass-through Z", [&](){
            auto params = &z_pass_through_filter_.params_;
            addUi("Enable", [&](){
                return ui::Checkbox("##value", &params->enable);
            });
            addUi("Range", [&](){
                return ui::DragFloatRange2("##value", &params->min, &params->max, 0.05f);
            });
        });
        addFilter("Voxel filter", [&](){
            addUi("Enable", [&](){
                return ui::Checkbox("##value", &voxel_filter_.params_.enable);
            });
            addUi("Voxel size", [&](){
                return ui::DragFloat("##value", &voxel_filter_.params_.size, 0.001f, 0.0f);
            });
        });
        addFilter("Statistical outlier removal", [&](){
            addUi("Enable", [&](){
                return ui::Checkbox("##value", &sor_filter_.params_.enable);
            });
            addUi("MeanK", [&](){
                return ui::InputInt("##value", &sor_filter_.params_.mean_k, 1);
            });
            addUi("StddevMulThresh", [&](){
                return ui::InputFloat("##value", &sor_filter_.params_.stddev_mul_threshold, 0.1f);
            });
        });

        ui::SetWindowPos(leftWindowPos);
        ui::SetWindowSize(vec2(kWindowWidth, 0));
        leftWindowPos.y += ui::GetWindowHeight() + kWindowSpacing;
    }

    if (visible_clouds_window_) {
        ui::ScopedWindow window("Clouds", kWindowFlags);

        if (ui::Button("Clear")) {
            grabbers_.clear();
            loading_progresses_.clear();
            grabber_selected_ = nullptr;
            updated_ = true;
        }

        if (grabber_selected_) {
            ui::SameLine();
            if (ui::Button("Remove")) {
                grabbers_.erase(grabber_selected_->path());
                loading_progresses_.erase(grabber_selected_->path());
                grabber_selected_ = nullptr;
                updated_ = true;
            }
        }

        if (grabber_selected_) {
            ui::SameLine();
            if (hidden_clouds_.find(grabber_selected_->path()) != hidden_clouds_.end()) {
                if (ui::Button("Show")) {
                    hidden_clouds_.erase(grabber_selected_->path());
                    updated_ = true;
                }
            } else {
                if (ui::Button("Hide")) {
                    hidden_clouds_.insert(grabber_selected_->path());
                    updated_ = true;
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

        ui::SetWindowPos(leftWindowPos);
        ui::SetWindowSize(vec2(kWindowWidth, 0));
        leftWindowPos.y += ui::GetWindowHeight() + kWindowSpacing;
    }
    {
        ui::ScopedWindow window("Connected devices", kWindowFlags);

        ui::ListBoxHeader("");
        for (auto pair : sensor_device_manager_.devices()) {
            ui::Selectable(pair.second->serial().c_str());
        }
        ui::ListBoxFooter();

        ui::SetWindowPos(leftWindowPos);
        ui::SetWindowSize(vec2(kWindowWidth, 0));
        leftWindowPos.y += ui::GetWindowHeight() + kWindowSpacing;
    }
    if (visible_player_window_) {
        ui::ScopedWindow window("Player", kWindowFlags);

        for (auto pair : loading_progresses_) {
            ui::TextUnformatted(pair.first.filename().c_str());
            auto prg = pair.second;
            ui::Columns(2);
            ui::ProgressBar(((float) prg[0]) / prg[1]);
            ui::NextColumn();
            ui::Text("%04d / %04d", (int) prg[0], (int) prg[1]);
            ui::SameLine();
            auto grabber = grabbers_[pair.first];
            if (grabber->isPlaying()) {
                if (ui::Button("Stop")) {
                    grabber->stop();
                }
            } else if (ui::Button("Play")) {
                grabber->start(on_cloud_updated_);
            }
            ui::Columns(1);
        }

        ui::SetWindowPos(rightWindowPos);
        ui::SetWindowSize(vec2(kWindowWidth, 0));
        rightWindowPos.y += ui::GetWindowHeight() + kWindowSpacing;
    }
    if (visible_debug_window_) {
        ui::ScopedWindow window("Information", kWindowFlags);
        ui::LabelText("FPS", "%f", getAverageFps());
        ui::LabelText("Cloud size", "%d", cloud_size_);
        ui::LabelText("Filtered", "%d", filtered_cloud_size_);
        ui::SetWindowPos(leftWindowPos);
        ui::SetWindowSize(vec2(kWindowWidth, 0));
        leftWindowPos.y += ui::GetWindowHeight() + kWindowSpacing;
    }

    if (updated_) {
        updatePointCloud();
    }
}

void CiPointCloudViewerApp::draw()
{
    gl::clear(bg_color_);

    gl::setMatrices(camera_);

    gl::pointSize(point_size_);

    if (visible_grid_) {
        grid_batch_->draw();
    }

    vert_batch_mutex_.lock();
    batch_->draw();
    vert_batch_mutex_.unlock();
}

CINDER_APP( CiPointCloudViewerApp, RendererGl, [](App::Settings *settings) {
    settings->setHighDensityDisplayEnabled();
    settings->setWindowSize(1280, 960);
})

