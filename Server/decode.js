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
    var message = String.fromCharCode.apply(null, bytes);
    message = message.split(/(?=[thpgfas])/g);
  
	for (i of message) {
	  switch(i[0]) {
		  case 't':
			  var str = i.slice(1);
			  temp = parseFloat(str);
			  break;
		  case 'h':
			  var str = i.slice(1);
			  humidity = parseFloat(str);
			  break;
		  case 'p':
			  var str = i.slice(1);
			  pressure = parseFloat(str);
			  break;
		  case 'g':
			  var str = i.slice(1);
			  gas = parseFloat(str);
			  break;
		  case 'f':
			  var str = i.slice(1);
			  frequency = parseFloat(str);
			  break;
		  case 'a':
			  var str = i.slice(1);
			  amplitude = parseFloat(str);
			  break;
		  case 's':
			  var str = i.slice(1);
			  seconds = parseFloat(str);
			  break;
	  }	
	}

	if (!isNaN(temp)) {
	  	jsonObject.temp = temp;
	}
	if (!isNaN(humidity)) {
	  	jsonObject.humidity = humidity;
	}
	if (!isNaN(pressure)) {
	  	jsonObject.pressure = pressure;
	}
	if (!isNaN(gas)) {
	  	jsonObject.gas = gas;
	}
	if (!isNaN(frequency)) {
	  	jsonObject.frequency = frequency;
	}
	if (!isNaN(amplitude)) {
	  	jsonObject.amplitude = amplitude;
	}
	if (!isNaN(seconds)) {
	  	jsonObject.seconds = seconds;
	}
	return JSON.stringify(jsonObject);
}
