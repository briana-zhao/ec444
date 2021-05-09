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

//--------------------------------------------------------------------------------------------------
var velocity = [];
var datalength = 20;
var intime = 0;

function readData(data) {

	data = data.split(" ");
	time = parseInt(data[0]);

	velocity.push({
		x: intime,
		y: parseFloat(data[1])
	});
	
	if (velocity.length > datalength) 
	{
		velocity.shift();
	}

    intime++;
	print_this = "Time: " + intime + " Velocity: " + data[1] + "m/s";
	console.log(print_this);
}

var velocityChart = {
	title: {
		text: "Speed"
	},
	axisX: {
		title: "Time",
		suffix: "s",
		ValueFormatString: "####",
		interval: 1
	},
	axisY: {
		title: "Speed",
		suffx: "m/s",
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
		name: "Speed",
		showInLegend: true,
		markerSize: 0,
		color: "red",
		lineThickness: 3,
		yValueFormatString: "#0.##m/s",
		xValueFormatString: "###0s",
		dataPoints: velocity
	}]
};


app.get('/', function(req, res){
	res.sendFile(__dirname + '/driving.html');
});

setInterval(function () {
	io.emit('velocity', velocityChart);
}, 1000);

io.on('connection', function (socket) {
	console.log('A user connected');

	io.emit('velocity', velocityChart);

	socket.on('sendValue', function (valueback) {       
		if (valueback != "") {
			msgBack = valueback;
			console.log('Value Back: ' + msgBack);
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

http.listen(8088, function () {
	console.log('listening on *:8088');
});