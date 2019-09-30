// +build !windows

package serial

// #include <termios.h>
// #include <unistd.h>
import "C"

// fixme: Maybe change to using syscall package + ioctl instead of cgo

import (
	"errors"
	"fmt"
	"math"
	"os"
	"syscall"
	"time"
	"unsafe"

	"golang.org/x/sys/unix"
)

const maxReadTimeout = (25 * time.Second) + (500 * time.Millisecond)

type impl struct {
	// We intentionally do not use an "embedded" struct so that we
	// don't export File
	c       *Config
	f       *os.File
	fd      uintptr
	st      C.struct_termios
}

var _ Port = (*impl)(nil)

func openPort(c Config) (p Port, err error) {
	f, err := os.OpenFile(c.Name, syscall.O_RDWR|syscall.O_NOCTTY|syscall.O_NONBLOCK, 0666)
	if err != nil {
		return
	}

	defer func() {
		if err != nil {
			_ = f.Close()
		}
	}()

	pt := &impl{
		c:  &c,
		f:  f,
		fd: f.Fd(),
	}

	if C.isatty(C.int(pt.fd)) != 1 {
		err = errors.New("serial: file is not a tty")
		return
	}

	if _, err = C.tcgetattr(C.int(pt.fd), &pt.st); err != nil {
		return
	}

	var speed C.speed_t
	switch c.Baud {
	case 230400:
		speed = C.B230400
	case 115200:
		speed = C.B115200
	case 57600:
		speed = C.B57600
	case 38400:
		speed = C.B38400
	case 19200:
		speed = C.B19200
	case 9600:
		speed = C.B9600
	case 4800:
		speed = C.B4800
	case 2400:
		speed = C.B2400
	case 1200:
		speed = C.B1200
	case 600:
		speed = C.B600
	case 300:
		speed = C.B300
	case 200:
		speed = C.B200
	case 150:
		speed = C.B150
	case 134:
		speed = C.B134
	case 110:
		speed = C.B110
	case 75:
		speed = C.B75
	case 50:
		speed = C.B50
	default:
		err = fmt.Errorf("serial: unknown baud rate %v", c.Baud)
		return
	}

	// by some bizarre input and output speeds set by separate calls
	if _, err = C.cfsetispeed(&pt.st, speed); err != nil {
		return
	}

	if _, err = C.cfsetospeed(&pt.st, speed); err != nil {
		return
	}

	// Turn off break interrupts, CR->NL, Parity checks, strip, and IXON
	pt.st.c_iflag &= ^C.tcflag_t(C.BRKINT | C.ICRNL | C.INPCK | C.ISTRIP | C.IXOFF | C.IXON | C.PARMRK)

	// Select local mode, turn off parity, set to 8 bits
	pt.st.c_cflag &= ^C.tcflag_t(C.CSIZE | C.PARENB)
	pt.st.c_cflag |= C.CLOCAL | C.CREAD

	// databits
	switch c.Size {
	case 5:
		pt.st.c_cflag |= C.CS5
	case 6:
		pt.st.c_cflag |= C.CS6
	case 7:
		pt.st.c_cflag |= C.CS7
	case 8:
		pt.st.c_cflag |= C.CS8
	default:
		err = ErrBadSize
		return
	}

	// Parity settings
	switch c.Parity {
	case ParityNone:
		// default is no parity
	case ParityOdd:
		pt.st.c_cflag |= C.PARENB
		pt.st.c_cflag |= C.PARODD
	case ParityEven:
		pt.st.c_cflag |= C.PARENB
		pt.st.c_cflag &= ^C.tcflag_t(C.PARODD)
	default:
		err = ErrBadParity
		return
	}
	// Stop bits settings
	switch c.StopBits {
	case Stop1:
		// as is, default is 1 bit
	case Stop2:
		pt.st.c_cflag |= C.CSTOPB
	default:
		err = ErrBadStopBits
		return
	}

	// Select raw mode
	pt.st.c_lflag &= ^C.tcflag_t(C.ICANON | C.ECHO | C.ECHOE | C.ISIG)
	pt.st.c_oflag &= ^C.tcflag_t(C.OPOST)

	// Disable RTS/CTS hardware flow control
	// pt.st.c_cflag &= ^C.tcflag_t(C.CRTSCTS)

	if err = pt.Flush(); err != nil {
		return
	}

	if err = pt.setTimeouts(1, posixTimeoutValue(c.timeout)); err != nil {
		return
	}

	r1, _, e := syscall.Syscall(syscall.SYS_FCNTL,
		pt.fd,
		uintptr(syscall.F_SETFL),
		uintptr(0))
	if e != 0 || r1 != 0 {
		err = fmt.Errorf("serial: clearing NONBLOCK syscall error: %s, %d", e, r1)
		return
	}

	p = pt

	return
}

func (p *impl) setTimeouts(vMin, vTime uint8) error {
	p.st.c_cc[C.VMIN] = C.cc_t(vMin)
	p.st.c_cc[C.VTIME] = C.cc_t(vTime)

	if _, err := C.tcsetattr(C.int(p.fd), C.TCSANOW, &p.st); err != nil {
		return err
	}

	return nil
}

// SetReadDeadline
func (p *impl) SetReadDeadline(t time.Duration) error {
	p.c.timeout = t

	return nil
}

func (p *impl) Read(b []byte) (n int, err error) {
	remaining := len(b)

	if remaining == 0 {
		return 0, ErrInvalidArg
	}

	defer func() {
		if p.c.DumpRx != nil && n > 0 {
			p.c.DumpRx(b[:n])
		}
	}()

	remainingTimeout := p.c.timeout

	if remainingTimeout == 0 {
		if err = p.setTimeouts(0, 0); err != nil {
			return
		}

		return p.f.Read(b)
	}

	offset := 0

	for remaining > 0 {
		toRead := remaining
		if toRead > 255 {
			toRead = 255
		}

		vMin := uint8(toRead)
		vTime := uint8(0)

		if remainingTimeout < MaxTimeout {
			t := remainingTimeout
			if t >= maxReadTimeout {
				t = maxReadTimeout
			}

			remainingTimeout -= t

			vTime = posixTimeoutValue(t)
		}

		if err = p.setTimeouts(vMin, vTime); err != nil {
			return
		}

		var nRead int
		nRead, err = p.f.Read(b[offset:])
		n += nRead
		remaining -= nRead
		offset += nRead
		if err != nil {
			return
		}
	}

	return
}

func (p *impl) Write(b []byte) (n int, err error) {
	if p.c.DumpTx != nil {
		p.c.DumpTx(b)
	}

	return p.f.Write(b)
}

// Discards data written to the port but not transmitted,
// or data received but not read
func (p *impl) Flush() error {
	_, err := C.tcflush(C.int(p.f.Fd()), C.TCIOFLUSH)
	return err
}

func (p *impl) Status() (n uint, err error) {
	var status uint
	if _, _, errno := unix.Syscall(
		unix.SYS_IOCTL,
		p.fd,
		uintptr(unix.TIOCMGET),
		uintptr(unsafe.Pointer(&status)),
	); errno != 0 {
		return status, errno
	} else {
		return status, nil
	}
}

func (p *impl) SetDTR(assert bool) (err error) {
	req := unix.TIOCMBIS
	if !assert {
		req = unix.TIOCMBIC
	}

	var m uint = unix.TIOCM_DTR
	if _, _, errno := unix.Syscall(
		unix.SYS_IOCTL,
		p.fd,
		uintptr(req),
		uintptr(unsafe.Pointer(&m)),
	); errno != 0 {
		return errno
	} else {
		return nil
	}
}

func (p *impl) SetRTS(assert bool) (err error) {
	req := unix.TIOCMBIS
	if !assert {
		req = unix.TIOCMBIC
	}

	var m uint = unix.TIOCM_RTS
	if _, _, errno := unix.Syscall(
		unix.SYS_IOCTL,
		p.fd,
		uintptr(req),
		uintptr(unsafe.Pointer(&m)),
	); errno != 0 {
		return errno
	} else {
		return nil
	}
}

func (p *impl) Close() (err error) {
	return p.f.Close()
}

// Converts the timeout values for Linux / POSIX systems
/*
 * http://man7.org/linux/man-pages/man3/termios.3.html
 * - Supports blocking read and read with timeout operations
 */
func posixTimeoutValue(duration time.Duration) (vtime uint8) {
	var readTimeoutInDeci int64

	if duration > 0 {
		// convert timeout to deciseconds as expected by VTIME
		readTimeoutInDeci = duration.Nanoseconds() / 1e6 / 100
		// capping the timeout
		if readTimeoutInDeci < 1 {
			// min possible timeout 1 deciseconds (0.1s)
			readTimeoutInDeci = 1
		} else if readTimeoutInDeci > math.MaxUint8 {
			// max possible timeout is 255 deciseconds (25.5s)
			readTimeoutInDeci = math.MaxUint8
		}
	}

	return uint8(readTimeoutInDeci)
}
