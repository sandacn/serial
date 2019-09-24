[![GoDoc](https://godoc.org/github.com/NotifAi/serial?status.svg)](http://godoc.org/github.com/NotifAi/serial)
[![Build Status](https://travis-ci.org/NotifAi/serial.svg?branch=master)](https://travis-ci.org/NotifAi/serial)

Serial
========
A Go package to allow you to read and write from the
serial port as a stream of bytes.

Details
-------
It aims to have the same API on all platforms, including windows.  As
an added bonus, the windows package does not use cgo, so you can cross
compile for windows from another platform.

You can cross compile with
   GOOS=windows GOARCH=386 go install github.com/NotifAi/serial

Currently there is very little in the way of configurability.  You can
set the baud rate.  Then you can Read(), Write(), or Close() the
connection.  By default Read() will block until at least one byte is
returned.  Write is the same.

Currently all ports are opened with 8 data bits, 1 stop bit, no
parity, no hardware flow control, and no software flow control.  This
works fine for many real devices and many faux serial devices
including usb-to-serial converters and Bluetooth serial ports.

You may Read() and Write() simultaneously on the same connection (from
different goroutines).

Usage
-----
```go
package main

import (
        "log"

        "github.com/tarm/serial"
)

func main() {
        c := &serial.Config{Name: "COM45", Baud: 115200}
        s, err := serial.OpenPort(c)
        if err != nil {
                log.Fatal(err)
        }
        
        n, err := s.Write([]byte("test"))
        if err != nil {
                log.Fatal(err)
        }
        
        buf := make([]byte, 128)
        n, err = s.Read(buf)
        if err != nil {
                log.Fatal(err)
        }
        log.Printf("%q", buf[:n])
}
```
