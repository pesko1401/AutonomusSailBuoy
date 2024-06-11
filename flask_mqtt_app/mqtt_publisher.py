import board
import busio
import digitalio
import adafruit_rfm9x
import logging
import paho.mqtt.client as mqtt
import json
import re
from datetime import datetime

# Configure logging
logging.basicConfig(filename='data.log', level=logging.INFO, format='%(asctime)s - %(message)s')

# LoRa configuration
RADIO_FREQ_MHZ = 433.0
CS = digitalio.DigitalInOut(board.CE1)
RESET = digitalio.DigitalInOut(board.D25)
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
rfm9x = adafruit_rfm9x.RFM9x(spi, CS, RESET, RADIO_FREQ_MHZ, baudrate=100000)
rfm9x.tx_power = 23

# MQTT configuration
mqtt_broker = "localhost"
mqtt_topic = "sensor/data"
mqtt_client = mqtt.Client()
mqtt_client.connect(mqtt_broker)

print("Waiting for packets...")

def is_valid_json(packet_text):
    try:
        json.loads(packet_text)
        return True
    except ValueError:
        return False

def strip_trailing_garbage(data):
    match = re.match(r'(\{.*\})(.*)', data)
    if match:
        return match.group(1)
    return data

def process_packet(packet):
    try:
        print("Full packet (hex):", packet.hex())
        packet_text = packet.decode('latin-1')
        print("Decoded packet text (latin-1):", packet_text)  # Debug print

        packet_text = strip_trailing_garbage(packet_text)
        print("Stripped packet text:", packet_text)  # Debug print

        if is_valid_json(packet_text):
            print(f"Valid JSON packet text: {packet_text}")

            data = json.loads(packet_text)
            sensor_name = data.get("type", "unknown")

            if sensor_name == "sst":
                sensor_value = data.get("value", "unknown")
                timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

                log_message = f"{timestamp}, {sensor_value}"
                logging.info(log_message)
                print(log_message)

                message = {
                    "data": data
                }

                mqtt_client.publish(mqtt_topic, json.dumps(message))
                print(f"Published message to MQTT broker: {json.dumps(message)}\n")
            else:
                print(f"Received non-sst data: {packet_text}")

        else:
            print(f"Invalid JSON: {packet_text}")
            logging.warning(f"Invalid JSON: {packet_text}")

    except Exception as e:
        print("Error decoding packet:", e)
        logging.error(f"Error decoding packet: {packet.hex()}", exc_info=True)

def main():
    while True:
        packet = rfm9x.receive(timeout=5.0, with_header=True)
        if packet is None:
            print("Received nothing! Listening again...")
        else:
            process_packet(packet)

if __name__ == "__main__":
    main()



























