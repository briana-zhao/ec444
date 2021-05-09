var Engine = require('tingodb')(),
    assert = require('assert');
const fs = require('fs');
const readline = require('readline');

// Create a new database
var db = new Engine.Db('.', {});
var collection = db.collection("smoke_database");


// Create a readable stream for the smoke.txt file
const readInterface = readline.createInterface({
    input: fs.createReadStream('smoke.txt'),
    console: false
});


var item = 0;
var heading;


// Read each line
readInterface.on('line', function(line) {
    if (item == 0) 
    {
      heading = line.split("\t"); //for the first line 'Time	ID	Smoke	Temp'
      item = item + 1;
    } 
    else 
    {
      var line_values= line.split("\t"); //split up the line
      var data = {};
      data[heading[0]] = line_values[0];  //Time
      data[heading[1]] = line_values[1];  //ID
      data[heading[2]] = line_values[2];  //Smoke
      data[heading[3]] = line_values[3];  //Temp

      // Insert data into the database
      collection.insert(data);
    }
});

// Query the database
readInterface.on('close', function(line) {
  
  collection.findOne({"ID": "5", "Time": "8000"}, function(err, item) {
    assert.equal(null, err);
    console.log('\nQuery for ID 5 at time 8000: ')
    console.log(item);
  })


  collection.findOne({"ID": "3", "Time": "4000"}, function(err, item) {
    assert.equal(null, err);
    console.log('\nQuery for ID 3 at time 4000: ')
    console.log(item);
  })

  collection.find().pretty();

});