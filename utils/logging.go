package utils

import (
	"fmt"
	"io"
	"os"
)

func CreateIOMultiWriterForLogFile(logFilePath string) (*os.File, io.Writer, error) {
	// Open the log file for writing.
	logFile, err := os.OpenFile(logFilePath, os.O_CREATE|os.O_WRONLY|os.O_APPEND, 0666)
	if err != nil {
		return nil, nil, fmt.Errorf("failed to open log file: %v", err)
	}

	// Create a MultiWriter that writes to both the log file and standard output.
	multiWriter := io.MultiWriter(logFile, os.Stdout)

	return logFile, multiWriter, nil
}
