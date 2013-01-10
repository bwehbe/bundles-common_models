require 'models/blueprints/sensors'

Dev::Sensors.device_type 'Stim300' do
    provides Srv::CalibratedIMUSensors
end

class Stim300::Task
    driver_for Dev::Sensors::Stim300

    # Additional information to allow for the transformer's automatic
    # configuration
    transformer do
        associate_frame_to_ports "stim300", "calibrated_sensors"
	associate_frame_to_ports "stim300", "incremental_velocity"
    end
end
