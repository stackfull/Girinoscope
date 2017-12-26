package org.hihan.girinoscope.comm;

import com.fazecast.jSerialComm.SerialPort;

import java.io.Closeable;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.LinkedList;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;
import java.util.regex.Pattern;

/**
 * Reading operations are semi-interruptible here. As long as nothing as been
 * read, it can be interrupted, but once something has been read, it continues
 * up to the line / buffer completion. This behavior is here to avoid to stop
 * reading a bunch of data sent by the Girino. It do it fast enough not to
 * bother us. The only delay we want to interrupt is when nothing is coming, per
 * instance when we wait for the trigger to happen. A crossover can still occur
 * - the trigger happening the same time the user cancel the operation - but it
 * is not likely to happen and the Girino doesn’t support a complex enough
 * protocol to prevent this kind of problem anyway. On the other hand, the
 * consequence is not fatal. We will read garbage the next time, display some
 * error to the user and move along.
 */
public class Serial implements Closeable {

    private static final Logger logger = Logger.getLogger(Serial.class.getName());

    /**
     * The port we're normally going to use. Port detection could be forced by
     * setting a property: -Dgnu.io.rxtx.SerialPorts=portName
     */
    private static final Pattern[] ACCEPTABLE_PORT_NAMES = {
            //
            Pattern.compile("/dev/tty\\.usbserial-.+"), // Mac OS X
            Pattern.compile("/dev/cu\\.wchusbserial.+"), // Mac OS X
            Pattern.compile("/dev/tty\\.usbmodem.+"), // Mac OS X
            Pattern.compile("/dev/ttyACM\\d+"), // Raspberry Pi
            Pattern.compile("/dev/ttyUSB\\d+"), // Linux
            // Pattern.compile("/dev/rfcomm\\d+"), // Linux Bluetooth
            Pattern.compile("COM\\d+"), // Windows
    };

    /** Milliseconds to block while waiting for port open. */
    private static final int TIME_OUT = 2000;

    /** Default bits per second for COM port. */
    private static final int DATA_RATE = 115200;

    /** Milliseconds to wait when no input is available. */
    private static final int READ_DELAY = 200;

    private SerialPort serialPort;

    /** The output stream to the port. */
    private InputStream input;

    /**
     * A BufferedReader which will be fed by a InputStreamReader converting the
     * bytes into characters making the displayed results codepage independent.
     */
    private OutputStream output;

    public Serial(SerialPort portId) throws Exception {
        connect(portId);
    }

    public void connect(SerialPort portId) throws Exception {
        serialPort = portId;
        portId.openPort();
        portId.setComPortTimeouts(SerialPort.TIMEOUT_READ_BLOCKING, TIME_OUT, TIME_OUT);
        portId.setComPortParameters(DATA_RATE, 8, SerialPort.ONE_STOP_BIT, SerialPort.NO_PARITY);
        portId.setFlowControl(SerialPort.FLOW_CONTROL_DISABLED);
        output = serialPort.getOutputStream();
        input = serialPort.getInputStream();
    }

    public static List<SerialPort> enumeratePorts() {
        List<SerialPort> ports = new LinkedList<SerialPort>();

        for (SerialPort port: SerialPort.getCommPorts()) {
            for (Pattern acceptablePortName : ACCEPTABLE_PORT_NAMES) {
                String portName = port.getSystemPortName();
                if (acceptablePortName.matcher(portName).matches()) {
                    ports.add(port);
                }
            }
        }
        return ports;
    }

    public String readLine() throws IOException {
        StringBuilder line = new StringBuilder();
        int length = 0;
        try {
            while (true) {
                int c;
                if ((input.available() > 0 || line.length() > 0) && (c = input.read()) >= 0) {
                    line.append((char) c);
                    ++length;
                    boolean eol = length >= 2 && line.charAt(length - 2) == '\r' && line.charAt(length - 1) == '\n';
                    if (eol) {
                        line.setLength(length - 2);
                        break;
                    }
                } else {
                    /*
                     * Sleeping here allows us to be interrupted (the serial
                     * input is not interruptible itself).
                     */
                    Thread.sleep(READ_DELAY);
                }
            }
        } catch (InterruptedException e) {
            logger.log(Level.FINE, "Read aborted");
            return null;
        }
        logger.log(Level.FINE, "< ({0})", line);
        return line.toString();
    }

    public int readBytes(byte[] buffer) throws IOException {
        int offset = 0;
        try {
            while (offset < buffer.length) {
                if (input.available() > 0 || offset > 0) {
                    int size = input.read(buffer, offset, buffer.length - offset);
                    if (size < 0) {
                        break;
                    }
                    offset += size;
                } else {
                    /*
                     * Sleeping here allows us to be interrupted (the serial
                     * input is not interruptible itself).
                     */
                    Thread.sleep(READ_DELAY);
                }
            }
        } catch (InterruptedException e) {
            logger.log(Level.FINE, "Read aborted");
            return -1;
        }
        logger.log(Level.FINE, "< {0} byte(s)", offset);
        return offset;
    }

    public void writeLine(String line) throws IOException {
        for (int i = 0; i < line.length(); ++i) {
            output.write(line.charAt(i));
        }
        output.flush();
        logger.log(Level.FINE, "> ({0})", line);
    }

    @Override
    public void close() {
        if (serialPort != null) {
            try {
                output.flush();
            } catch (IOException e) {
                logger.log(Level.WARNING, "When flushing output before closing serial.", e);
            }
            serialPort.closePort();
            serialPort = null;
        }
    }
}
