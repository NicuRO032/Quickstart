package org.firstinspires.ftc.teamcode.util;

import java.io.OutputStreamWriter;
import java.io.PrintWriter;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.LinkedHashMap;
import java.util.Locale;
import java.util.Map;

public class DesktopTelemetryServer {
    private final int port;
    private ServerSocket serverSocket;
    private Socket clientSocket;
    private PrintWriter writer;
    private Thread serverThread;
    private volatile boolean isRunning = false;

    // Buffer pentru datele de telemetrie stil Dashboard
    private final Map<String, String> telemetryMap = new LinkedHashMap<>();

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
                serverSocket.setReuseAddress(true);

                while (isRunning) {
                    Socket socket = serverSocket.accept();
                    socket.setTcpNoDelay(true);
                    socket.setKeepAlive(true);

                    synchronized (this) {
                        if (clientSocket != null && !clientSocket.isClosed()) {
                            try { clientSocket.close(); } catch (Exception ignored) {}
                        }
                        if (writer != null) {
                            writer.close();
                        }

                        clientSocket = socket;
                        writer = new PrintWriter(new OutputStreamWriter(clientSocket.getOutputStream()), true);
                    }
                }
            } catch (Exception ignored) {}
        }, "DesktopTelemetryServerThread");

        serverThread.setDaemon(true);
        serverThread.start();
    }

    // Trimitere pozitie robot
    public synchronized void sendPose(double x, double y, double headingDeg) {
        if (writer != null) {
            if (writer.checkError()) {
                cleanupClient();
            } else {
                writer.println(String.format(Locale.US, "POSE,%.3f,%.3f,%.3f", x, y, headingDeg));
            }
        }
    }

    // Adaugă o cheie și o valoare în pachetul curent (stil packet.put)
    public synchronized void put(String key, Object value) {
        telemetryMap.put(key, String.valueOf(value));
    }

    // Trimite tot pachetul de date adunat și curăță buffer-ul
    public synchronized void sendTelemetry() {
        if (writer != null && !telemetryMap.isEmpty()) {
            if (writer.checkError()) {
                cleanupClient();
                return;
            }

            StringBuilder builder = new StringBuilder("DATA");
            for (Map.Entry<String, String> entry : telemetryMap.entrySet()) {
                builder.append(";").append(entry.getKey()).append("=").append(entry.getValue());
            }

            writer.println(builder.toString());
            telemetryMap.clear();
        }
    }

    private void cleanupClient() {
        if (writer != null) {
            writer.close();
            writer = null;
        }
        if (clientSocket != null && !clientSocket.isClosed()) {
            try { clientSocket.close(); } catch (Exception ignored) {}
            clientSocket = null;
        }
    }

    public synchronized void stop() {
        isRunning = false;
        try {
            cleanupClient();
            if (serverSocket != null && !serverSocket.isClosed()) {
                serverSocket.close();
                serverSocket = null;
            }
        } catch (Exception ignored) {}
    }
}