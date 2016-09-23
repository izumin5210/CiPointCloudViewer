//
// Created by Masayuki IZUMI on 7/19/16.
//

#include "impl/ViewParamsImpl.h"
#include "action/ViewParamsAction.h"

class ViewParamsImpl : public ViewParams {
public:
  INJECT(ViewParamsImpl(std::shared_ptr<Dispatcher> dispatcher))
    : dispatcher_         (dispatcher)
    , look_at_            (0, 0.5, 0)
    , eye_point_          (4.0, 2.0, -4.0)
    , bg_color_           (cinder::Color8u(0x11, 0x11, 0x11))
    , point_size_         (1.0f)
    , enable_full_screen_ (false)
    , grid_               (ViewParams::Grid::RECTANGULAR)
  {
    initializeConnections();
  }

  glm::vec3 look_at() const override {
    return look_at_;
  }

  glm::vec3 eye_point() const override {
    return eye_point_;
  }

  cinder::Color bg_color() const override {
    return bg_color_;
  }

  float point_size() const override {
    return point_size_;
  }

  bool is_full_screen() const override {
    return enable_full_screen_;
  }

  Grid grid() const override {
    return grid_;
  }


private:
  std::shared_ptr<Dispatcher> dispatcher_;

  glm::vec3 look_at_;
  glm::vec3 eye_point_;
  ci::Color bg_color_;
  float point_size_;
  bool enable_full_screen_;
  Grid grid_;

  void initializeConnections() {
    namespace ph = std::placeholders;
    addConnection(dispatcher_->connect<UpdateCameraParamsAction>(std::bind(&ViewParamsImpl::onCameraParamsUpdate, this, ph::_1)));
    addConnection(dispatcher_->connect<UpdateBgColorAction>(std::bind(&ViewParamsImpl::onBgColorUpdate, this, ph::_1)));
    addConnection(dispatcher_->connect<UpdatePointSizeAction>(std::bind(&ViewParamsImpl::onPointSizeUpdate, this, ph::_1)));
    addConnection(dispatcher_->connect<ToggleFullScreenAction>(std::bind(&ViewParamsImpl::onFullScreenToggle, this, ph::_1)));
    addConnection(dispatcher_->connect<ChangeGridAction>(std::bind(&ViewParamsImpl::onGridChange, this, ph::_1)));
  }

  void onCameraParamsUpdate(const UpdateCameraParamsAction &action) {
    eye_point_ = action.eye_point;
    look_at_ = action.look_at;
    emit();
  }

  void onBgColorUpdate(const UpdateBgColorAction &action) {
    bg_color_ = action.bg_color;
  }

  void onPointSizeUpdate(const UpdatePointSizeAction &action) {
    point_size_ = action.size;
  }

  void onFullScreenToggle(const ToggleFullScreenAction &action) {
    enable_full_screen_ = action.enable;
  }

  void onGridChange(const ChangeGridAction &action) {
    grid_ = action.grid;
  }
};


fruit::Component<
    fruit::Required<Dispatcher>,
    ViewParams
>
getViewParamsImplComponent() {
  return fruit::createComponent().bind<ViewParams, ViewParamsImpl>();
}
