import sys
import json
from time import time

from influxdb_client import InfluxDBClient
from influxdb_client.client.write_api import SYNCHRONOUS

# used for tests
jsonObj = '{"amplitude":80,"counter":1,"frequency":4440,"gas":8065.16,"humidity":32.23,"pressure":728.67,"temperature":33.95}'
#jsonObj = '{"counter":1,"gas":18065.16,"humidity":23,"temperature":37.95}'
appName = "SoundMeasurementSystem"
devEUI = b'G\xcbU\x80\x00\x17\x00:'.hex()

def dbWrite(appName, devEUI, jsonObject):
    data = json.loads(jsonObject)
    bucket = "test/"
    
    if appName != 'SoundMeasurementSystem':
        return False

    for i in data:
        if type(data.get(i)) != float and type(data.get(i)) != int:
            return False

    for i in data:
        if i == "temperature" or i == "gas" or i == "humidity" or i == "pressure":
            data[i] = float(data[i])
    try:    
        _client = InfluxDBClient.from_config_file("config.ini")
        _write_api = _client.write_api(write_options=SYNCHRONOUS)
        #_write_api.write(bucket=bucket, record={"measurement": appName,
        #                                        "tags": {"devEUI": devEUI},
        #                                        "fields": data,
        #                                        "time": int(time() * 10**9)})
        _write_api.write(bucket=bucket, record={"measurement": appName,
                                                "tags": {"devEUI": devEUI},
                                                "fields": data})
    except:
        print(sys.exc_info()[0])
    finally:
        _write_api.__del__()
        _client.__del__()

    return True

if __name__ == '__main__':
    print(dbWrite(appName, devEUI, jsonObj))
