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
                // SO_REUSEADDR permite redeschiderea rapida a portului fara erori de "Address already in use"
                serverSocket = new ServerSocket(port);
                serverSocket.setReuseAddress(true);

                while (isRunning) {
                    // Blocking accept()
                    Socket socket = serverSocket.accept();

                    // Configurari pentru socket-ul clientului
                    socket.setTcpNoDelay(true); // Trimite pachetele de telemetrie imediat (fara buffering Nagle)
                    socket.setKeepAlive(true);

                    synchronized (this) {
                        // Inchidem conexiunea veche daca exista una activa
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
            } catch (Exception ignored) {
                // Exceptie normala cand serverSocket este inchis din stop()
            }
        }, "DesktopTelemetryServerThread");

        serverThread.setDaemon(true);
        serverThread.start();
    }

    public synchronized void sendPose(double x, double y, double headingDeg) {
        if (writer != null) {
            // Verificam daca clientul s-a deconectat sau daca canalul a intampinat o eroare
            if (writer.checkError()) {
                cleanupClient();
            } else {
                writer.println(String.format(Locale.US, "%.3f,%.3f,%.3f", x, y, headingDeg));
            }
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
        } catch (Exception ignored) {
            // Cleanup exceptions ignored
        }
    }
}