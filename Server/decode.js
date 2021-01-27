// Decode decodes an array of bytes into an object.
//  - fPort contains the LoRaWAN fPort number
//  - bytes is an array of bytes, e.g. [225, 230, 255, 0]
//  - variables contains the device variables e.g. {"calibration": "3.5"} (both the key / value are of type string)
// The function must return an object, e.g. {"temperature": 22.5}
function Decode(fPort, bytes, variables) {
    var jsonObject = new Object();
    var temp;
    var humidity;
    var pressure;
    var gas;
    var frequency;
    var amplitude;
    var seconds;

    // FLAGS used to find what is in payload
    var TEMP_FLAG = 64;    // 01000000
    var HUM_FLAG = 32;     // 00100000
    var PRES_FLAG = 16;    // 00010000
    var GAS_FLAG = 8;      // 00001000
    var FREQ_FLAG = 4;     // 00000100
    var AMP_FLAG = 2;      // 00000010
    var S_FLAG = 1;        // 00000001

    var offset = 0;

    if (bytes[0] & TEMP_FLAG) {
        jsonObject.temperature = (bytes[offset+1]<<24) | (bytes[offset+2]<<16) | (bytes[offset+3]<<8) | (bytes[offset+4]);
        offset += 4;
    }
    if (bytes[0] & HUM_FLAG) {
        jsonObject.humidity = (bytes[offset+1]<<24) | (bytes[offset+2]<<16) | (bytes[offset+3]<<8) | (bytes[offset+4]);
        offset += 4;
    }
    if (bytes[0] & PRES_FLAG) {
        jsonObject.pressure = (bytes[offset+1]<<24) | (bytes[offset+2]<<16) | (bytes[offset+3]<<8) | (bytes[offset+4]);
        offset += 4;
    }
    if (bytes[0] & GAS_FLAG) {
    jsonObject.gas = (bytes[offset+1]<<24) | (bytes[offset+2]<<16) | (bytes[offset+3]<<8) | (bytes[offset+4]);
        offset += 4;
    }
    if (bytes[0] & FREQ_FLAG) {
      jsonObject.frequency = (bytes[offset+1]<<8) | (bytes[offset+2]);
        offset += 2;
    }
    if (bytes[0] & AMP_FLAG) {
        jsonObject.amplitude = (bytes[offset+1]<<8) | (bytes[offset+2]);
        offset += 2;
    }
    if (bytes[0] & S_FLAG) {
        jsonObject.seconds = (bytes[offset+1]<<8) | (bytes[offset+2]);
    }

    return jsonObject;
}