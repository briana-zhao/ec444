const SerialPort = require('serialport')
const Readline = require('@serialport/parser-readline')
const port = new SerialPort('COM3', {baudRate: 115200});

const parser = port.pipe(new Readline({ delimiter: '\r\n' }))
parser.on('data', readSerialInput)

function readSerialInput(data)
{
    console.log(data);
}