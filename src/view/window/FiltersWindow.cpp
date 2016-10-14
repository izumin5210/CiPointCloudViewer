//
// Created by Masayuki IZUMI on 9/29/16.
//

#include "Signal.h"
#include "view/window/FiltersWindow.h"

namespace view {
namespace window {

FiltersWindow::FiltersWindow(
  const std::string name,
  const int width,
  const int spacing,
  const ImGuiWindowFlags flags,
  const std::shared_ptr<Clouds> &clouds
)
  : Window(name, width, spacing, flags)
  , clouds_(clouds)
{}

void FiltersWindow::drawImpl() {
  addFilter("Pass-through X", std::bind(&FiltersWindow::addPassThroughFilterX, this));
  addFilter("Pass-through Y", std::bind(&FiltersWindow::addPassThroughFilterY, this));
  addFilter("Pass-through Z", std::bind(&FiltersWindow::addPassThroughFilterZ, this));
  addFilter("Voxel filter", std::bind(&FiltersWindow::addVoxelFilter, this));
  addFilter("Statistical outlier removal", std::bind(&FiltersWindow::addSorFilter, this));
#ifdef USE_NITE2
  addFilter("Display only humans", std::bind(&FiltersWindow::addUsersThroughFilter, this));
#endif
}

void FiltersWindow::addPassThroughFilterX() {
  addPassThroughFilter("x", clouds_->x_pass_through_filter_params());
}

void FiltersWindow::addPassThroughFilterY() {
  addPassThroughFilter("y", clouds_->y_pass_through_filter_params());
}

void FiltersWindow::addPassThroughFilterZ() {
  addPassThroughFilter("z", clouds_->z_pass_through_filter_params());
}

void FiltersWindow::addVoxelFilter() {
  auto params = clouds_->voxel_filter_params();
  bool updated = false;
  updated = updated || addFilterParam("Enable", [&]() {
    return ui::Checkbox("##value", &params.enable);
  });
  updated = updated || addFilterParam("Voxel size", [&]() {
    return ui::DragFloat("##value", &params.size, 0.001f, 0.0f);
  });
  if (updated) {
    Signal<Clouds::UpdateVoxelFilterParamsAction>::emit({params});
  }
}

void FiltersWindow::addSorFilter() {
  auto params = clouds_->sor_filter_params();
  bool updated = false;
  updated = updated || addFilterParam("Enable", [&]() {
    return ui::Checkbox("##value", &params.enable);
  });
  updated = updated || addFilterParam("MeanK", [&]() {
    return ui::InputInt("##value", &params.mean_k, 1);
  });
  updated = updated || addFilterParam("StddevMulThresh", [&]() {
    return ui::InputFloat("##value", &params.stddev_mul_threshold, 0.1f);
  });
  if (updated) {
    Signal<Clouds::UpdateStatisticalOutlierRemovalFilterParamsAction>::emit({params});
  }
}

#ifdef USE_NITE2
void FiltersWindow::addUsersThroughFilter() {
  auto enable = clouds_->enable_users_through_filter();
  bool updated = false;
  updated = updated || addFilterParam("Enable", [&]() {
    return ui::Checkbox("##value", &enable);
  });
  if (updated) {
    Signal<Clouds::UpdateUsersThroughFitlerParamsAction>::emit({enable});
  }
}
#endif

void FiltersWindow::addFilter(const char *label, std::function<void()> definition) {
  if (ui::TreeNode(label)) {
    ui::Columns(2);
    definition();
    ui::TreePop();
    ui::Columns(1);
  }
}

bool FiltersWindow::addFilterParam(const char *label, std::function<bool()> updater) {
  ui::PushID(label);
  ui::AlignFirstTextHeightToWidgets();
  ui::Bullet();
  ui::TextUnformatted(label);
  ui::NextColumn();
  ui::PushItemWidth(-1);
  bool updated = updater();
  ui::PopItemWidth();
  ui::NextColumn();
  ui::PopID();
  return updated;
}

void FiltersWindow::addPassThroughFilter(const std::string &name, PassThroughFilterParams params) {
  bool updated = false;
  updated = updated || addFilterParam("Enable", [&]() {
    return ui::Checkbox("##value", &params.enable);
  });
  updated = updated || addFilterParam("Range", [&]() {
    return ui::DragFloatRange2("##value", &params.min, &params.max, 0.05f);
  });
  if (updated) {
    Signal<Clouds::UpdatePassThroughFilterParamsAction>::emit({name, params});
  }
}

}
}
