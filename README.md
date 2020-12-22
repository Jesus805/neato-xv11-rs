# Neato XV-11 in Rust

A multi-threaded driver library for the Neato XV-11 LIDAR.

## Usage

### TODO: Description

```
use neato_xv11;
use std::sync::mpsc::channel;

// Create a message channel.
let (message_tx, message_rx) = channel();
// Create a command channel.
let (command_tx, command_rx) = channel();

thread::spawn(move || {
    neato_xv11::run("/dev/serial0", message_tx, command_rx);
});
```

### TODO: Description

```
// Receive a message from the LIDAR.
match self.message_rx.try_recv() {
    Ok(r) => {
        match r {
            Ok(message) => println!(message),
            Err(error) => println!(error),
        }
    }
    Err(e) => {
        match e {
            // Data not ready, do nothing.
            TryRecvError::Empty => println!("No items detected yet"),
            // Channel disconnected, stop running.
            TryRecvError::Disconnected => println!("Disconnected"),
        }
    }
}
```

## License

Copyright © 2020 Jesus Bamford
Distributed under the [MIT License](https://github.com/Jesus805/neatoxv11-rs/blob/master/LICENSE).
