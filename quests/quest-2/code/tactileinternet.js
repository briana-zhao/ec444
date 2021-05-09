var app = require('express')();
var http = require('http').Server(app);
var io = require('socket.io')(http);

var ir_rangefinder = [];
var ultrasonic = [];
var thermistor = [];

app.get('/', function(req, res){
	res.sendFile(__dirname + '/tactileinternet.html');
});
  
//Call the chartEvent every second
setInterval(function(){ io.emit('chartEvent', chartOptions);}, 1000);

//Whenever someone connects
io.on('connect', function(socket){
	console.log('A user connected');
	io.emit('chartEvent', chartOptions);

	//Whenever someone disconnects
	socket.on('disconnect', function(){ console.log('A user disconnected'); });
});

  
// Set up to read from serial port (from the ESP32)
const SerialPort = require('serialport')
const Readline = require('@serialport/parser-readline')
const port = new SerialPort('COM3', {baudRate: 115200});

const parser = port.pipe(new Readline({ delimiter: '\r\n' }))
parser.on('data', readSerialData)

var initial_time = null;
var datalength = 20;
var print_this = "Time: ";

//Parse the data and store in appropriate arrays
function readSerialData(data) {
	data = data.split(" ");
	time = parseInt(data[0]);
	if (initial_time == null) {
		initial_time = time;
	}
	time = time - initial_time;

	thermistor.push({
		x: time,
		y: parseFloat(data[1])
	});
	ultrasonic.push({
		x: time,
		y: parseFloat(data[2])
	});
	ir_rangefinder.push({
		x: time,
		y: parseFloat(data[3])
	});

	if (thermistor.length > datalength) {
		ir_rangefinder.shift();
		ultrasonic.shift();
		thermistor.shift();
	}

	print_this = "Time: " + time + " Temperature: " + data[1] + "ºC" + " Ultrasonic Sensor: " + data[2] + "m" + " IR Sensor: " + data[3] + "m";
	console.log(print_this);	//Print the readings
}

//Format the chart
var chartOptions = {
	title: {
		text: "Tactile Internet",
		fontSize: 34
	},
	axisX: {
		title: "Time",
		suffix: "s",
		ValueFormatString: "####",
		interval: 1
	},
	axisY: [ {
		title: "Temperature",
		suffix: "ºC"
	},
	{
		title: "Distance",
		suffix: "m",
		minimum: 0,
		maximum: 3
	}],
	toolTip: {
		shared: true
	},
	legend: {
		cursor: "pointer",
		verticalAlign: "top",
		horizontalAlign: "center",
		dockInsidePlotArea: true,
		itemclick: toggleDataSeries
	},
	data: [{
		type:"line",
		name: "IR Rangefinder",
		showInLegend: true,
		lineThickness: 3,
		dataPoints: ir_rangefinder,
		yValueFormatString: "#0.##m",
		xValueFormatString: "###0s",
		axisYIndex: 1
	},
	{
		type: "line",
		name: "Ultrasonic",
		showInLegend: true,
		lineThickness: 3,
		dataPoints: ultrasonic,
		yValueFormatString: "#0.##m",
		xValueFormatString: "###0s",
		axisYIndex: 1
	},
	{
		type: "line",
		name: "Thermistor",
		showInLegend: true,
		lineThickness: 3,
		dataPoints: thermistor,
		yValueFormatString: "#0.##ºC",
		xValueFormatString: "###0s",
		axisYIndex: 0
	}]
};

//http://localhost:8080
http.listen(8080, function(){
  console.log('listening on localhost:8080');
});


function toggleDataSeries(e){
	if (typeof(e.dataSeries.visible) === "undefined" || e.dataSeries.visible) {
		e.dataSeries.visible = false;
	} else{
		e.dataSeries.visible = true;
	}
	chart.render();
}