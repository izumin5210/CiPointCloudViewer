//
// Created by Masayuki IZUMI on 7/20/16.
//

#include "impl/io/CloudDataSourcesImpl.h"
#include "action/CloudDataSourcesAction.h"

namespace io {

  class CloudDataSourcesImpl : public CloudDataSources {
  public:
    INJECT(CloudDataSourcesImpl(
        const std::shared_ptr<Dispatcher> dispatcher
    ))
      : dispatcher_(dispatcher)
    {
      initializeConnections();
    };

    std::map<Key, std::shared_ptr<SequentialPcdPlayer>> sequential_pcd_players() const override {
      return sequential_pcd_players_;
    };

  private:
    const std::shared_ptr<Dispatcher> dispatcher_;
    std::map<Key, std::shared_ptr<SequentialPcdPlayer>> sequential_pcd_players_;

   void addConnection(Dispatcher::Connection connection) {
      // TODO: not yet implemented
    }

    void initializeConnections() {
      addConnection(dispatcher_->connect<OpenPcdFilesDirectoryAction>(
          std::bind(&CloudDataSourcesImpl::onPcdFilesDirectoryOpen, this, std::placeholders::_1)
      ));
    }

    void onPcdFilesDirectoryOpen(const OpenPcdFilesDirectoryAction &action) {
      sequential_pcd_players_[action.path] = std::make_shared<SequentialPcdPlayer>(dispatcher_, action.path);
    }
  };
}

fruit::Component<fruit::Required<Dispatcher>, io::CloudDataSources>
getCloudDataSourcesImplComponent() {
  return fruit::createComponent()
      .bind<io::CloudDataSources, io::CloudDataSourcesImpl>();
}
