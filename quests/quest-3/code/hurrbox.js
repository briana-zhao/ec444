var app = require('express')();
var http = require('http').Server(app);
var io = require('socket.io')(http);

var dgram = require('dgram');

// Port and IP
var PORT = 3333;
var HOST = "192.168.0.27";

// Create socket
var server = dgram.createSocket('udp4');

// Create server
server.on('listening', function () {
	var address = server.address();
	console.log('UDP Server listening on ' + address.address + ":" + address.port);
});

var msgBack = 'Ok!';

// On connection, print out received message
server.on('message', function (msg, remote) {

	readData(msg.toString());
	console.log(remote.address + ':' + remote.port + ' - ' + msg);

	// Send Ok acknowledgement
	server.send(msgBack, remote.port, remote.address, function (error) {
		if (error) {
			console.log('MEH!');
		}
		else {
			console.log('Sent: ' + msgBack);
			msgBack = 'Ok!';
		}
	});

});

// Bind server to port and IP
server.bind(PORT, HOST);

//--------------------------------------------------------------------------------------------------------------//
var temperature = [];
var battery_voltage = [];
var vibration_X = [];
var vibration_Y = [];
var vibration_Z = [];
var datalength = 20;
var print_this = "Time: ";

function readData(data) {

	data = data.split(" ");
	time = parseInt(data[0]);

	temperature.push({
		x: time,
		y: parseFloat(data[1])
	});
	battery_voltage.push({
		x: time,
		y: parseFloat(data[2])
	});
	vibration_X.push({
		x: time,
		y: parseFloat(data[3])
	});
	vibration_Y.push({
		x: time,
		y: parseFloat(data[4])
	});
	vibration_Z.push({
		x: time,
		y: parseFloat(data[5])
	});
	
	if (temperature.length > datalength) 
	{
		temperature.shift();
		battery_voltage.shift();
		vibration_X.shift();
		vibration_Y.shift();
		vibration_Z.shift();
	}

	print_this = "Time: " + time + " Temperature: " + data[1] + "ºC" + " Battery voltage: " + data[2] + "V" + " Vibration x: " + data[3] + "G's" + " Vibration y: " + data[4] + "G's" + " Vibration z: " + data[5] + "G's";
	console.log(print_this);
}

var batteryVoltageChart = {
	title: {
		text: "Battery Level"
	},
	axisX: {
		title: "Time",
		suffix: "s",
		ValueFormatString: "####",
		interval: 1
	},
	axisY: {
		title: "Voltage",
		suffx: "V",
		valueFormatString: "#0.##"
	},
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
		type: "line",
		name: "Voltage",
		showInLegend: true,
		markerSize: 0,
		color: "red",
		lineThickness: 3,
		yValueFormatString: "#0.##V",
		xValueFormatString: "###0s",
		dataPoints: battery_voltage
	}]
};


var temperatureChart = {
	title: {
		text: "Temperature"
	},
	axisX: {
		title: "Time",
		suffix: "s",
		ValueFormatString: "####",
		interval: 1
	},
	axisY: {
		title: "Temperature",
		suffix: "ºC",
		valueFormatString: "#0.##"
	},
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
		type: "line",
		name: "Temperature",
		showInLegend: true,
		markerSize: 0,
		color: "green",
		lineThickness: 3,
		yValueFormatString: "#0.##ºC",
		xValueFormatString: "###0s",
		dataPoints: temperature
	}]
};

var vibrationXChart = {
	title: {
		text: "Vibration in X"
	},
	axisX: {
		title: "Time",
		suffix: "s",
		ValueFormatString: "####",
		interval: 1
	},
	axisY: {
		title: "Vibration",
		suffix: "G's",
		valueFormatString: "#0.##"
	},
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
		type: "line",
		name: "Vibration in X",
		lineThickness: 3,
		showInLegend: true,
		markerSize: 0,
		yValueFormatString: "#0.##G's",
		xValueFormatString: "###0s",
		dataPoints: vibration_X
	}]
};

var vibrationYChart = {
	title: {
		text: "Vibration in Y"
	},
	axisX: {
		title: "Time",
		suffix: "s",
		ValueFormatString: "####",
		interval: 1
	},
	axisY: {
		title: "Vibration",
		suffix: "G's",
		valueFormatString: "#0.##"
	},
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
		type: "line",
		name: "Vibration in Y",
		lineThickness: 3,
		showInLegend: true,
		markerSize: 0,
		yValueFormatString: "#0.##G's",
		xValueFormatString: "###0s",
		dataPoints: vibration_Y
	}]
};

var vibrationZChart = {
	title: {
		text: "Vibration in Z"
	},
	axisX: {
		title: "Time",
		suffix: "s",
		ValueFormatString: "####",
		interval: 1
	},
	axisY: {
		title: "Vibration",
		suffix: "G's",
		valueFormatString: "#0.##"
	},
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
		type: "line",
		name: "Vibration in Z",
		lineThickness: 3,
		showInLegend: true,
		markerSize: 0,
		yValueFormatString: "#0.##G's",
		xValueFormatString: "###0s",
		dataPoints: vibration_Z
	}]
};

app.get('/', function(req, res){
	res.sendFile(__dirname + '/hurrbox.html');
});

setInterval(function () {
	io.emit('vibrationX', vibrationXChart);
	io.emit('vibrationY', vibrationYChart);
	io.emit('vibrationZ', vibrationZChart);
	io.emit('temperature', temperatureChart);
	io.emit('batteryVoltage', batteryVoltageChart);
}, 1000);

io.on('connection', function (socket) {
	console.log('A user connected');

	io.emit('vibrationX', vibrationXChart);
	io.emit('vibrationY', vibrationYChart);
	io.emit('vibrationZ', vibrationZChart);
	io.emit('temperature', temperatureChart);
	io.emit('batteryVoltage', batteryVoltageChart);

	socket.on('ledValue', function (ledcontrol) {         //Need later for LED stuff
		if (ledcontrol != null) {
			msgBack = ledcontrol;
			console.log('LED Intensity: ' + msgBack);
		}
	});

	socket.on('disconnect', function (){ console.log('A user disconnected'); });
});

function toggleDataSeries(e) {
	if (typeof (e.dataSeries.visible) === "undefined" || e.dataSeries.visible) {
		e.dataSeries.visible = false;
	} else {
		e.dataSeries.visible = true;
	}
	chart.render();
}

http.listen(8080, function () {
	console.log('listening on *:8080');
});