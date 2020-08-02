# Neato XV-11 in Rust

A driver library for the Neato XV-11 LIDAR

## Usage
Simply import the driver and run it

```
use neato_xv11::NeatoXV11Lidar;

let mut lidar = NeatoXV11Lidar::new();
lidar.run("/dev/serial0");
```

## License

Copyright © 2020 Jesus Bamford
Distributed under the [MIT License](https://github.com/Jesus805/neatoxv11-rs/blob/master/LICENSE).
