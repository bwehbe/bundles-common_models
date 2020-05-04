require 'common_models/models/services/dvl'
require 'common_models/models/devices/gazebo/entity'
module CommonModels
    module Devices
        module Gazebo
            device_type 'Dvl' do
                provides Entity
                provides Services::Dvl
            end
        end
    end
end
