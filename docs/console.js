//
// from google example
//
//  https://chromium.googlesource.com/apps/libapps/+/refs/heads/main/hterm/doc/embed.md
//

function initContent(io) {
    const ver = lib.resource.getData('hterm/changelog/version');
    const date = lib.resource.getData('hterm/changelog/date');
    const pkg = `hterm ${ver} (${date})`;
    /* eslint-disable quotes */
    io.println("\r\n\
                   Welcome to SPI Hub Device!\r\n\
         Press F11 to go fullscreen to use all shortcuts.\r\n\
                     Running " + "V0.17 8/10/2021 " + ".\r\n\
    ");
}

// Load translations if available.
lib.registerInit('load messages', async () => {
});

function setupHterm() {

    'use strict';

    var port;
    var data = {
        "cmd": "",
    };

    hterm.defaultStorage = new lib.Storage.Local();

    var term = new hterm.Terminal();
    let textEncoder = new TextEncoder();
    let input = new Array;
    var encoded = new Uint8Array;

    term.onTerminalReady = function () {
        const io = this.io.push();
        function printPrompt() {
            io.print(
                '\x1b[38:2:51:105:232mspihub>' +
                '\x1b[0m ');
        }

        function printHub(bytedata) {
            if (port !== undefined) {
                port.send(textEncoder.encode(bytedata)).catch(error => {
                    io.println('Send error: ' + error);
                });
            }
        }
         
        io.onVTKeystroke = str => {
            switch (str) {
                case '\r':
                    // msgpack
                    data.cmd = input.join("");
                    encoded = msgpack.pack(data, true);
                    //console.log(encoded);
                    printHub(encoded);
                    io.println('');
                     input = new Array;
                    printPrompt();
                    break;
                case '\x7f':
                    // \x08 = backspace, \x1b[K = 'Erase in line'.
                    io.print('\x08\x1b[K');
                    input.pop();
                    break;
                default:
                    io.print(str);
                    input.push(str);
                    break;
            }
        };
        io.sendString = str => {
            io.print(str);
         //   printHub(str);
       };

        initContent(io);
        printPrompt();
        this.setCursorVisible(true);
        this.keyboard.bindings.addBinding('F11', 'PASS');
        this.keyboard.bindings.addBinding('Ctrl+R', 'PASS');
    };

    term.decorate(document.querySelector('#terminal'));
    term.setWidth(80);
    term.setHeight(50);
    term.installKeyboard();
    // Useful for console debugging.
    window.term_ = term;
 
    term.contextMenu.setItems([
        { name: 'Terminal Reset', action: () => term.reset() },
        { name: 'Terminal Clear', action: () => term.clear() },
        { name: hterm.ContextMenu.SEPARATOR },
        {
            name: 'Homepage', action: function () {
                lib.f.openWindow(
                    'https://chromium.googlesource.com/apps/libapps/+/HEAD/hterm/README.md',
                    '_blank');
            }
        },
        {
            name: 'FAQ', action: function () {
                lib.f.openWindow('https://goo.gl/muppJj', '_blank');
            }
        },
    ]);

    let connectButton = document.querySelector('#connect');

    function connect() {
        //term.io.println('Connecting to ' + port.device_.productName + '...');
        port.connect().then(() => {
            console.log(port);
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

    connectButton.addEventListener('click', function () {
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

}

