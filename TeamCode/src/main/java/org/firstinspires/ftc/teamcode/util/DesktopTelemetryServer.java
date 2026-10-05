package org.firstinspires.ftc.teamcode.util;

import java.io.OutputStreamWriter;
import java.io.PrintWriter;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.Locale;

public class DesktopTelemetryServer {
    private final int port;
    private ServerSocket serverSocket;
    private Socket clientSocket;
    private PrintWriter writer;
    private Thread serverThread;
    private volatile boolean isRunning = false;

    public DesktopTelemetryServer() {
        this(5555);
    }

    public DesktopTelemetryServer(int port) {
        this.port = port;
    }

    public synchronized void start() {
        if (isRunning) return;
        isRunning = true;

        serverThread = new Thread(() -> {
            try {
                serverSocket = new ServerSocket(port);
                while (isRunning) {
                    // accept() blocks waiting for connection, outside synchronized block
                    Socket socket = serverSocket.accept();

                    synchronized (this) {
                        if (clientSocket != null && !clientSocket.isClosed()) {
                            clientSocket.close();
                        }
                        clientSocket = socket;
                        writer = new PrintWriter(new OutputStreamWriter(clientSocket.getOutputStream()), true);
                    }
                }
            } catch (Exception ignored) {
                // Exception caught when serverSocket is closed on stop()
            }
        }, "DesktopTelemetryServerThread");

        serverThread.setDaemon(true);
        serverThread.start();
    }

    public synchronized void sendPose(double x, double y, double headingDeg) {
        if (writer != null && !writer.checkError()) {
            writer.println(String.format(Locale.US, "%.3f,%.3f,%.3f", x, y, headingDeg));
        }
    }

    public synchronized void stop() {
        isRunning = false;
        try {
            if (writer != null) {
                writer.close();
                writer = null;
            }
            if (clientSocket != null && !clientSocket.isClosed()) {
                clientSocket.close();
                clientSocket = null;
            }
            if (serverSocket != null && !serverSocket.isClosed()) {
                serverSocket.close();
                serverSocket = null;
            }
        } catch (Exception ignored) {
            // Cleanup exceptions ignored
        }
    }
}