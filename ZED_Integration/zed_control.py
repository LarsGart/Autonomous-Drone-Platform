from zed_model import ZedModel


zed = ZedModel()

print('zed',zed)
print('zed.init_params',zed.init_params)
print('zed.info',zed.info)
print('zed.cam_model', zed.cam_model)
print('zed.sensors', zed.sensors)

print('zed.get_sensor_configuration',zed.get_sensor_configuration())
print('zed.get_sensor_data for accelerometer',zed.get_sensor_data('accelerometer'))
print('zed.get_sensor_data for gyroscope',zed.get_sensor_data('gyroscope'))
print('zed.get_quaternion',zed.get_quaternion())
print('zed.get_euler',zed.get_euler())