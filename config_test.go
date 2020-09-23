package serial

import (
	"testing"

	"github.com/stretchr/testify/require"
	"gopkg.in/yaml.v3"
)

func TestConfig(t *testing.T) {
	const stream = `
parity: none
`

	var c Config

	err := yaml.Unmarshal([]byte(stream), &c)
	require.NoError(t, err)
}
