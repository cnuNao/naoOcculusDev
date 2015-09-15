/**
 * Sample udp client. Sends random message to test udp.c
 * 
 * @author juanvallejo
 * @date 6/25/15
 */

var socket = require('dgram').createSocket('udp4');

var usrinput = 'Empty message';

// populate user input
if(process.argv[2]) {
	
	usrinput = '';

	for(var i = 2; i < process.argv.length; i++) {
		usrinput += process.argv[i] + ' ';
	}

	// remove last space
	usrinput = usrinput.substring(0, usrinput.length - 1);

}

var message = new Buffer(usrinput, 'utf-8');
socket.send(message, 0, message.length, 14552, '0.0.0.0', function(err, bytes) {

	if(err) {
		console.log('error sending message> ' + err);
	} else {
		console.log('message sent...');
	}

	socket.close();

});