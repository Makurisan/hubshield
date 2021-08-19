(function() {
  'use strict';

  lib.init();

 // hterm.defaultStorage = new lib.Storage.Local();

  var port;

  let textEncoder = new TextEncoder();

  let term = new hterm.Terminal();

  term.onTerminalReady = () => {
    console.log('Terminal ready.');
    let io = term.io.push();

    io.onVTKeystroke = str => {
      if (port !== undefined) {
        port.send(textEncoder.encode(str)).catch(error => {
          term.io.println('Send error: ' + error);
        });
      }
    };

    io.sendString = str => {
      if (port !== undefined) {
        port.send(textEncoder.encode(str)).catch(error => {
          term.io.println('Send error: ' + error);
        });
      }
    };

      term.decorate(document.querySelector('#terminal'));
      term.setWidth(80);
      term.setHeight(50);
      term.installKeyboard();
  };

  document.addEventListener('DOMContentLoaded', event => {
    let connectButton = document.querySelector('#connect');

      

    function connect() {
      term.io.println('Connecting to ' + port.device_.productName + '...');
      port.connect().then(() => {
        console.log(port);
        term.io.println('Connected.');
        connectButton.textContent = 'Disconnect';
        port.onReceive = data => {
          let textDecoder = new TextDecoder();
          term.io.print(textDecoder.decode(data));
        }
        port.onReceiveError = error => {
          term.io.println('Receive error: ' + error);
        };
      }, error => {
        term.io.println('Connection error: ' + error);
      });
    };

    connectButton.addEventListener('click', function() {
      if (port) {
        port.disconnect();
        connectButton.textContent = 'Connect';
        port = null;
      } else {
        serial.requestPort().then(selectedPort => {
          port = selectedPort;
          connect();
        }).catch(error => {
          term.io.println('Connection error: ' + error);
        });
      }
    });

    serial.getPorts().then(ports => {
      if (ports.length == 0) {
        term.io.println('No devices found.');
      } else {
        port = ports[0];
        connect();
      }
    });
  });
})();
