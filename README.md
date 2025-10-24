# UniCAN Library

A C++ library for representing CAN (Controller Area Network) buses with support for both CAN 2.0 and CAN FD protocols.

## Features

- Support for both CAN 2.0 (up to 8 bytes) and CAN FD (up to 64 bytes) frames
- Standard (11-bit) and extended (29-bit) frame IDs
- Remote Transmission Request (RTR) frame support
- Fluent builder pattern API
- Comprehensive Doxygen documentation

## Documentation

To generate the documentation using Doxygen:

```bash
# Install Doxygen (if not already installed)
sudo apt-get install doxygen

# Generate documentation
doxygen Doxyfile

# View the documentation
open docs/html/index.html
```

