from flask import Flask, render_template, send_file, jsonify, request
import paho.mqtt.client as mqtt
import json
import threading

app = Flask(__name__)

# Dictionary to store the latest five messages for each sensor
sensor_data = {
    'sst': [],
    'temperature': [],
    'humidity': [],
    'pressure': [],
    'magnetometer': [],
    'accelerometer': []
}

# MQTT settings
mqtt_broker = "localhost"
mqtt_topic = "sensor/data"

def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    client.subscribe(mqtt_topic)

def on_message(client, userdata, msg):
    payload = msg.payload.decode()
    try:
        data = json.loads(payload)
        sensor_type = data['data']['type']
        timestamp = data['data']['timestamp']
        value = data['data']['value']

        if sensor_type in sensor_data:
            sensor_data[sensor_type].append((timestamp, value))
            if len(sensor_data[sensor_type]) > 5:
                sensor_data[sensor_type].pop(0)

        print(f"Received {sensor_type} data: {value}")

    except json.JSONDecodeError:
        print(f"Failed to decode JSON payload: {payload}")

def setup_mqtt():
    mqtt_client = mqtt.Client()
    mqtt_client.on_connect = on_connect
    mqtt_client.on_message = on_message
    mqtt_client.connect(mqtt_broker, 1883, 60)
    mqtt_thread = threading.Thread(target=mqtt_client.loop_forever)
    mqtt_thread.start()
    return mqtt_client

@app.route('/')
def index():
    return render_template('index.html', sensor_data=sensor_data)

@app.route('/sensor_data')
def sensor_data_page():
    sensor_type = request.args.get('type', 'temperature')
    return render_template('sensor_data.html', sensor_type=sensor_type)

@app.route('/download_log')
def download_log():
    sensor = request.args.get('sensor')
    if sensor:
        filename = f"{sensor}_data.log"
    else:
        filename = "data.log"
    return send_file(filename, as_attachment=True)

@app.route('/sensor_data.json')
def get_sensor_data():
    return jsonify(sensor_data)

if __name__ == '__main__':
    mqtt_client = setup_mqtt()
    app.run(host='0.0.0.0', port=5000, debug=True)









