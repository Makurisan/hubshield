(function() {
  'use strict';

  lib.init();

 // hterm.defaultStorage = new lib.Storage.Local();

  var port;

  let textEncoder = new TextEncoder();

  let term = new hterm.Terminal();

    // Load translations if available.
  lib.registerInit('load messages', async () => {
      // Try to load the messages database from nassh.  This isn't strictly needed
      // (so if it fails, it should be fine), but does help when testing locally.
      // $1 here means we search the user's language.  Change it to 'en' or another
      // specific language to test those specifically.
      const lang = '$1';
      await hterm.messageManager.findAndLoadMessages(
          lib.f.getURL(`../../nassh/_locales/${lang}/messages.json`));
  });


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
