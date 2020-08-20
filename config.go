package serial

import (
	"errors"
	"time"

	"gopkg.in/yaml.v3"
)

// Config contains the information needed to open a serial port.
//
// Currently few options are implemented, but more may be added in the
// future (patches welcome), so it is recommended that you create a
// new config addressing the fields by name rather than by order.
//
// For example:
//
//    c0 := &serial.Config{Name: "COM45", Baud: 115200, ReadTimeout: time.Millisecond * 500}
// or
//    c1 := new(serial.Config)
//    c1.Name = "/dev/tty.usbserial"
//    c1.Baud = 115200
//    c1.ReadTimeout = time.Millisecond * 500
//
type Config struct {
	Name string `yaml:"name,omitempty"`
	Baud int    `yaml:"baud,omitempty"`
	// Size is the number of data bits. If 0, DefaultSize is used.
	Size DataSize `yaml:"dataBits"`
	// Parity is the bit to use and defaults to ParityNone (no parity bit).
	Parity Parity `yaml:"parity"`
	// StopBits number of stop bits to use. Default is 1 (1 stop bit).
	StopBits StopBits     `yaml:"stopBits"`
	DumpRx   func([]byte) `yaml:"-"`
	DumpTx   func([]byte) `yaml:"-"`
	timeout  time.Duration
}

const DefaultSize = 8 // Default value for Config.Size

type DataSize byte
type StopBits byte
type Parity byte

const (
	MaxTimeout = time.Duration(1<<63 - 1)
)

const (
	Stop1     StopBits = 1
	Stop1Half StopBits = 15
	Stop2     StopBits = 2
)

const (
	ParityNone  Parity = 'N'
	ParityOdd   Parity = 'O'
	ParityEven  Parity = 'E'
	ParityMark  Parity = 'M' // parity bit is always 1
	ParitySpace Parity = 'S' // parity bit is always 0
)

func (p *Parity) UnmarshalYAML(node *yaml.Node) error {
	var res Parity
	switch node.Value {
	case "":
		fallthrough
	case "none":
		res = ParityNone
	case "odd":
		res = ParityOdd
	case "even":
		res = ParityEven
	case "mark":
		res = ParityMark
	case "space":
		res = ParitySpace
	default:
		return errors.New("invalid parity value")
	}

	*p = res

	return nil
}

func (s *StopBits) UnmarshalYAML(node *yaml.Node) error {
	var res StopBits

	switch node.Value {
	case "":
		fallthrough
	case "1":
		res = Stop1
	case "1.5":
		res = Stop1Half
	case "2":
		res = Stop2
	default:
		return errors.New("invalid stop bits value")
	}

	*s = res

	return nil
}

func (d *DataSize) UnmarshalYAML(node *yaml.Node) error {
	var res DataSize

	switch node.Value {
	case "7":
		res = 7
	case "":
		fallthrough
	case "8":
		res = DefaultSize
	case "9":
		res = 9
	default:
		return errors.New("invalid data size value")
	}

	*d = res

	return nil
}
