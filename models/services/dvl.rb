import_types_from 'base'

module CommonModels
    module Services
        data_service_type 'Dvl' do
            output_port 'velocity','/base/Vector3d'
        end
    end
end
