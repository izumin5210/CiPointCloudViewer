//
// Created by Masayuki IZUMI on 9/29/16.
//

#ifndef CIPOINTCLOUDVIEWERAPP_FILTERSWINDOW_H
#define CIPOINTCLOUDVIEWERAPP_FILTERSWINDOW_H

#include "Clouds.h"
#include "Window.h"

namespace view {
namespace window {

class FiltersWindow : public Window {
public:
  FiltersWindow(
    const std::string name,
    const int width,
    const int spacing,
    const ImGuiWindowFlags flags,
    const std::shared_ptr<Clouds> &clouds
  );


protected:
  void drawImpl() override;


private:
  using PassThroughFilterParams = filter::PassThroughFilter<Clouds::PointT>::Params;
  const std::shared_ptr<Clouds> clouds_;

  void addPassThroughFilterX();
  void addPassThroughFilterY();
  void addPassThroughFilterZ();
  void addVoxelFilter();
  void addSorFilter();
#ifdef USE_NITE2
  void addUsersThroughFilter();
#endif
  void addFilter(const char *label, std::function<void()> definition);
  bool addFilterParam(const char *label, std::function<bool()> updater);
  void addPassThroughFilter(const std::string &name, PassThroughFilterParams params);
};

}
}

#endif //CIPOINTCLOUDVIEWERAPP_FILTERSWINDOW_H
