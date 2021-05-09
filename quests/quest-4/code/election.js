var Engine = require('tingodb')(),
    assert = require('assert');
const fs = require('fs');
const readline = require('readline');
var app = require('express')();
var http = require('http').Server(app);
var io = require('socket.io')(http);
var dgram = require('dgram');


//var received_message;

// Port and IP
var PORT = 3333;
var HOST = "192.168.0.17";

// Create socket
var server = dgram.createSocket('udp4');

// Create server
server.on('listening', function () {
	var address = server.address();
	console.log('UDP Server listening on ' + address.address + ":" + address.port);
});

var msgBack = "";

// On connection, print out received message
server.on('message', function (msg, remote) {

    //received_message = msg;
	readData(msg.toString());
	console.log(remote.address + ':' + remote.port + ' - ' + msg);

	// Send Ok acknowledgement
	server.send(msgBack, remote.port, remote.address, function (error) {
		if (error) {
			console.log('MEH!');
		}
		else {
			console.log('Sent: ' + msgBack);
			msgBack = 'Okay!';
		}
	});

});

// Bind server to port and IP
server.bind(PORT, HOST);

//--------------------------------------------------------------------------------------------------------
var red_votes = 0;
var green_votes = 0;
var blue_votes = 0;
var print_this;
var item = 0;
var heading;
var db = new Engine.Db('.', {});
var collection = db.collection("votes_database");

var current_entry = 0;

var queryThis = null;
var queryResult = 0;


function readData(data) {

    if (item == 0) 
    {
        firstline = "Time Red Green Blue";
        heading = firstline.split(" ");
        var first_entry = {};
        first_entry[heading[0]] = 0;
        first_entry[heading[1]] = 0;
        first_entry[heading[2]] = 0;
        first_entry[heading[3]] = 0;
        item = item + 1;
    } 
    else 
    {
        var line_values= data.split(" "); //split up the line
        var insert_data = {};
        insert_data[heading[0]] = current_entry;
        insert_data[heading[1]] = line_values[0];  //Red
        insert_data[heading[2]] = line_values[1];  //Green
        insert_data[heading[3]] = line_values[2];  //Blue

        // Insert data into the database
        collection.insert(insert_data);

        current_entry++;
        console.log(current_entry);
    }

	data = data.split(" ");


    red_votes = data[0];
    green_votes = data[1];
    blue_votes = data[2];

	//print_this = "Red Votes: " + red_votes + " Green Votes: " + green_votes + " Blue votes: " + blue_votes;

    collection.findOne({"Time": current_entry}, function(err, item) {
        assert.equal(null, err);
        console.log('\nQuery: ')
        console.log(item.Red);
        switch(queryThis) {
            case 'Red' :
                console.log('\nQuery red: ')
                console.log(item.Red); 
                queryResult = item.Red;
                break;
            case 'Green' :
                console.log('\nQuery green: ')
                console.log(item.Green);
                queryResult = item.Green;
                break;
            case 'Blue' :
                console.log('\nQuery blue: ')
                console.log(item.Blue);
                queryResult = item.Blue;
               break;
            default :
               break;
         }

      })
}



app.get('/', function(req, res){
	res.sendFile(__dirname + '/election.html');
});

setInterval(function () {
	io.emit('queryEmit', queryResult);
}, 1000);

io.on('connection', function (socket) {
	console.log('A user connected');

	io.emit('queryEmit', queryResult);

	socket.on('queryValue', function (querycontrol) {  
		if (querycontrol != null) {
			queryThis = querycontrol;
			//console.log('Query is: ' + queryThis);
		}
	});

    socket.on('resetValue', function (resetcontrol) { 
        if (resetcontrol == 1)
        {
            msgBack = 'reset';
        }
		//console.log('Reset is: ' + msgBack);
	});

	socket.on('disconnect', function (){ console.log('A user disconnected'); });
});
http.listen(8080, function () {
	console.log('listening on *:8080');
});