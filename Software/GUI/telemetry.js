export const data={
  "timestamp": "2024-06-29 15:30:45",
  "system_status": "ONLINE",
  "adcs": {
    "pitch": 12.3,
    "roll": -5.7,
    "yaw": 80.2,
    "gps": {
      "latitude": 31.2001,
      "longitude": 29.9187,
      "altitude": 407.2
    },
    "light_sensors": {
      "face_a": 842,
      "face_b": 156
    }
  },
  "eps": {
    "battery_voltage": 7.4,
    "battery_percentage": 50,
    "current": 0.32,
    "solar_power": 2.1,
    "charging_status": "NOT Charging",
    "boost_output": 12.0,
    "temperature": 80
  },
  "obc": {
    "data_logging": "ACTIVE",
    "storage_usage": 67,
    "rtc_sync": "OK",
    "i2c_status": "Connected",
    "command_queue": 3
  },
  "comms": {
    "rf_link": "STRONG",
    "signal_strength": -85,
    "packets_sent": 1000,
    "packets_received": 1198,
    "last_command": "CAPTURE_IMAGE"
  },
  "payload": {
    "status": "CAPTURING",
    "images_today": 47,
    "last_image_size": 2.3,
    "ai_classification": {
      "result": "Ocean Surface",
      "confidence": 94.2
    }
  }
}
