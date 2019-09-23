// +build !windows

package serial

// #include <termios.h>
// #include <unistd.h>
import "C"

// TODO: Maybe change to using syscall package + ioctl instead of cgo

import (
	"errors"
	"fmt"
	"os"
	"syscall"
	"time"
	"unsafe"

	// "unsafe"

	"golang.org/x/sys/unix"
)

type impl struct {
	// We intentionally do not use an "embedded" struct so that we
	// don't export File
	f  *os.File
	fd uintptr
}

var _ Port = (*impl)(nil)

func openPort(name string, baud int, databits byte, parity Parity, stopbits StopBits, readTimeout time.Duration) (p Port, err error) {
	f, err := os.OpenFile(name, syscall.O_RDWR|syscall.O_NOCTTY|syscall.O_NONBLOCK, 0666)
	if err != nil {
		return
	}

	defer func() {
		if err != nil {
			_ = f.Close()
		}
	}()

	fd := f.Fd()
	if C.isatty(C.int(fd)) != 1 {
		err = errors.New("serial: file is not a tty")
		return
	}

	var st C.struct_termios
	if _, err = C.tcgetattr(C.int(fd), &st); err != nil {
		return
	}

	var speed C.speed_t
	switch baud {
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
		err = fmt.Errorf("unknown baud rate %v", baud)
		return
	}

	if _, err = C.cfsetispeed(&st, speed); err != nil {
		return
	}

	if _, err = C.cfsetospeed(&st, speed); err != nil {
		return
	}

	// Turn off break interrupts, CR->NL, Parity checks, strip, and IXON
	st.c_iflag &= ^C.tcflag_t(C.BRKINT | C.ICRNL | C.INPCK | C.ISTRIP | C.IXOFF | C.IXON | C.PARMRK)

	// Select local mode, turn off parity, set to 8 bits
	st.c_cflag &= ^C.tcflag_t(C.CSIZE | C.PARENB)
	st.c_cflag |= C.CLOCAL | C.CREAD

	// databits
	switch databits {
	case 5:
		st.c_cflag |= C.CS5
	case 6:
		st.c_cflag |= C.CS6
	case 7:
		st.c_cflag |= C.CS7
	case 8:
		st.c_cflag |= C.CS8
	default:
		err = ErrBadSize
		return
	}

	// Parity settings
	switch parity {
	case ParityNone:
		// default is no parity
	case ParityOdd:
		st.c_cflag |= C.PARENB
		st.c_cflag |= C.PARODD
	case ParityEven:
		st.c_cflag |= C.PARENB
		st.c_cflag &= ^C.tcflag_t(C.PARODD)
	default:
		err = ErrBadParity
		return
	}
	// Stop bits settings
	switch stopbits {
	case Stop1:
		// as is, default is 1 bit
	case Stop2:
		st.c_cflag |= C.CSTOPB
	default:
		err = ErrBadStopBits
		return
	}
	// Select raw mode
	st.c_lflag &= ^C.tcflag_t(C.ICANON | C.ECHO | C.ECHOE | C.ISIG)
	st.c_oflag &= ^C.tcflag_t(C.OPOST)

	// set blocking / non-blocking read
	/*
	*	http://man7.org/linux/man-pages/man3/termios.3.html
	* - Supports blocking read and read with timeout operations
	 */
	vmin, vtime := posixTimeoutValues(readTimeout)
	st.c_cc[C.VMIN] = C.cc_t(vmin)
	st.c_cc[C.VTIME] = C.cc_t(vtime)

	if _, err = C.tcsetattr(C.int(fd), C.TCSANOW, &st); err != nil {
		return
	}

	r1, _, e := syscall.Syscall(syscall.SYS_FCNTL,
		fd,
		uintptr(syscall.F_SETFL),
		uintptr(0))
	if e != 0 || r1 != 0 {
		err = fmt.Errorf("clearing NONBLOCK syscall error: %s, %d", e, r1)
		return
	}

	return &impl{f: f, fd: fd}, nil
}

func (p *impl) Read(b []byte) (n int, err error) {
	return p.f.Read(b)
}

func (p *impl) Write(b []byte) (n int, err error) {
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
