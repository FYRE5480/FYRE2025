package frc.robot.util;

import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.WebSocket;
import java.net.http.WebSocket.Listener;
import java.time.Duration;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.CompletionStage;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;

import com.google.gson.Gson;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonPrimitive;

/**
 * A websocket client to interface with Astrolabe's server running on a(n) RPI(s).
 */
public class CameraWebsocketClient {
    private String ip = "ws://10.42.0.118:50000";
    private WebSocket webSocket;
    private double rotation;
    private volatile String latestReply = "";
    private CountDownLatch messageLatch = new CountDownLatch(1);
    private final int TIMEOUT;

    /** Simple class representing a color object. */
    public static class Color {
        public double red;
        public double green;
        public double blue;
        public double difference;
        public double blur;
    }

    /** Simple class that contains information that Astrolabe returns. */
    public static class Info {
        public String cameraName;
        public String identifier;
        public double horizontalFocalLength;
        public double verticalFocalLength;
        public int height;
        public int horizontalResolutionPixels;
        public int verticalResolutionPixels;
        public int processingScale;
        public double tiltAngleRadians;
        public double horizontalFieldOfViewRadians;
        public double verticalFieldOfViewRadians;
        public List<Color> colorList;
        public int activeColor;
        public String fullString;
    }

    /** Simple class that represents an Apriltag. */
    public static class Apriltag {
        public String tagId;
        public double[] position;
        public double[] orientation;
        public double distance;
        public double horizontalAngle;
        public double verticalAngle;
        public String fullString;
    }

    /** Represents a information about a piece on the feild. */
    public static class Piece {
        public double distance;
        public double angle; // radians
        public double[] center;
        public double pieceAngle; // radians
    }

    public CameraWebsocketClient() {
        TIMEOUT = 5000;
    }

    public CameraWebsocketClient(String ip, int timeout) {
        TIMEOUT = timeout;
        this.ip = ip;
    }

    /**
     * Attempts a connection to the websocket server.

     * @return isConnected - whether or not there was a successful connection
     */
    public boolean setupConnection() {
        // This function sets up the connection to the websocket server. It returns true if the connection was successful and false if it was not.
        // Call this at any time if you want to reconnect to the server.
        try {
            HttpClient client = HttpClient.newHttpClient();
            webSocket = client.newWebSocketBuilder().connectTimeout(Duration.ofSeconds(5))
                    .buildAsync(URI.create(ip), new WebSocketListener(this))
                    .join();
        } catch (Exception e) {
            e.printStackTrace();
            return false;
        }
        return isConnected();
    }

    /**
     * Returns the status of the websocked connection.

     * @return boolean - whether or not the websocket is connected
     */
    public boolean isConnected() {
        return webSocket != null
            && webSocket.isOutputClosed() == false
            && webSocket.isInputClosed() == false;
    }

    public void sendMessage(String message) {
        webSocket.sendText(message, true);
    }

    /**
     * The callback for when a new message is received.

     * @param newMessage - the message from the server
     */
    public void onMessage(String newMessage) {
        // System.out.println("Received message: " + newMessage);
        this.latestReply = newMessage;
        messageLatch.countDown();
    }

    public String getMessage() {
        return latestReply;
    }

    /**
     * Tries to get the next response from the websocket server.

     * @return latestReply - the response from the server
     */
    public String getLatestReply() {
        try {
            messageLatch.await(TIMEOUT, TimeUnit.MILLISECONDS); // Wait for up to 5 seconds
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        return latestReply;
    }

    public void clear() {
        latestReply = "";
    }

    public JsonObject decodeJson(String jsonString) {
        Gson gson = new Gson();
        return gson.fromJson(jsonString, JsonObject.class);
    }

    public double getRotation() {
        return rotation;
    }

    public void setRotation(double rotation) {
        this.rotation = rotation;
    }

    /**
     * Requests info from the websocket server and returns it's reply.

     * @return Info - the info that the server returned
     */
    public Info getInfo() {
        try {
            sendMessage("info");
            String newMessage = getLatestReply();
            return getInfoFromString(newMessage);
        } catch (Exception e) {
            return new Info();
        }
    }

    /**
     * Parses a string as a Json object and returns it as an Info object.

     * @param pMessage - the stringified json to parse
     * @return info - the Info object that was represented in the json string
     */
    private Info getInfoFromString(String pMessage) {
        if (pMessage == null){
            return null;
        }
        try {
            JsonObject json = decodeJson(pMessage);
            Info info = new Info();
            info.cameraName = json.get("cam_name").getAsString();
            info.identifier = json.get("identifier").getAsString();
            info.horizontalFocalLength = json.get("horizontal_focal_length").getAsDouble();
            info.verticalFocalLength = json.get("vertical_focal_length").getAsDouble();
            info.height = json.get("height").getAsInt();
            info.horizontalResolutionPixels = json.get("horizontal_resolution_pixels").getAsInt();
            info.verticalResolutionPixels = json.get("vertical_resolution_pixels").getAsInt();
            info.processingScale = json.get("processing_scale").getAsInt();
            info.tiltAngleRadians = json.get("tilt_angle_radians").getAsDouble();
            info.horizontalFieldOfViewRadians = json.get("horizontal_field_of_view_radians").getAsDouble();
            info.verticalFieldOfViewRadians = json.get("vertical_field_of_view_radians").getAsDouble();
            info.activeColor = json.get("active_color").getAsInt();
            info.colorList = new ArrayList<>();
            for (var colorJson : json.getAsJsonArray("color_list")) {
                Color color = new Color();
                JsonObject colorObject = colorJson.getAsJsonObject();
                color.red = colorObject.get("red").getAsDouble();
                color.green = colorObject.get("green").getAsDouble();
                color.blue = colorObject.get("blue").getAsDouble();
                color.difference = colorObject.get("difference").getAsDouble();
                color.blur = colorObject.get("blur").getAsDouble();
                info.colorList.add(color);
            }
            info.fullString = pMessage;
            return info;
        } catch (Exception e) {
            System.out.println("Error getting info");
            e.printStackTrace();
            return new Info();
        }
    }

    /**
     * Saves the array of colors passed in to be saved on the server

     * @param colors - the array of colors that will be sent over to the server
     * @return success - whether or not the command was successful or not
     */
    public boolean saveColors(Color[] colors) {
        String outgoingString = "sp -values";

        JsonArray colorArray = new JsonArray();
        for (Color color : colors) {
            JsonObject colorObject = new JsonObject();
            colorObject.add("red", new JsonPrimitive(color.red));
            colorObject.add("green", new JsonPrimitive(color.green));
            colorObject.add("blue", new JsonPrimitive(color.blue));
            colorObject.add("difference", new JsonPrimitive(color.difference));
            colorObject.add("blur", new JsonPrimitive(color.blur));
            
            colorArray.add(colorObject);
        }

        // TODO: do some error check for this
        sendMessage(outgoingString + colorArray.toString());

        // TODO: after error checks, return true or false based on the success of the command
        return true;
    }

    /**
     * Switches the color that the server will look for when doing piece detection.

     * @param index - the index in the color array 
     * @return boolean - whether the color was switched
     */
    public boolean switchColors(int index) {
        sendMessage("sc -new_color=" + index);

        // TODO: error checks and retun success
        return true;
    }

    /**
     * Requests information from the coprocessor and returns it as a Piece object.

     * @return piece - information about the peice, if available 
     */
    public Piece getPiece() {
        try {
            sendMessage("fp");
            String newMessage = getLatestReply();
            return getPieceFromString(newMessage);
        } catch (Exception e) {
            e.printStackTrace();
            if (setupConnection()) {
                sendMessage("fp");
                String newMessage = getLatestReply();
                return getPieceFromString(newMessage);
            }
            return null;
        }
    }


    /**
     * Makes a request to the websocket server for apriltag information and returns its response.

     * @return aprilTags - a list of Apriltags that we got from the server
     */
    public List<Apriltag> getApriltags() {
        try {
            sendMessage("fa");
            String newMessage = getLatestReply();
            return getApriltagsFromString(newMessage);
        } catch (Exception e) {
            e.printStackTrace();
            if (setupConnection()) {
                sendMessage("fa");
                String newMessage = getLatestReply();
                return getApriltagsFromString(newMessage);
            }
            return new ArrayList<>();
        }
    }

    private double parseJsonElementAsDouble(JsonElement element) {
        // TODO: Implement this with other functions
        if (element != null && element.isJsonPrimitive() && element.getAsJsonPrimitive().isNumber()) {
            return element.getAsDouble();
        } else {
            return Double.NaN;
        }
    }

    private Piece getPieceFromString(String pMessage) {
        if (pMessage == null || pMessage.contains("error")) return null;

        try {
            JsonObject json = new Gson().fromJson(pMessage, JsonObject.class);
            Piece piece = new Piece();

            if (json != null && json.size() > 0) {
                piece.distance = parseJsonElementAsDouble(json.get("distance"));
                piece.angle = parseJsonElementAsDouble(json.get("angle"));
                piece.center = new double[] {
                    parseJsonElementAsDouble(json.get("center").getAsJsonArray().get(0)),
                    parseJsonElementAsDouble(json.get("center").getAsJsonArray().get(1))
                };
                piece.pieceAngle = parseJsonElementAsDouble(json.get("piece_angle"));
                return piece;
            } else return null;
        } catch (Exception e) {
            e.printStackTrace();
            return new Piece();
        }
    }

    private List<Apriltag> getApriltagsFromString(String pMessage) {
        if (pMessage == null) {
            return null;
        }
        // System.out.println(pMessage);
        try {
            JsonArray jsonArray = new Gson().fromJson(pMessage, JsonArray.class);
            List<Apriltag> apriltags = new ArrayList<>();

            if (jsonArray != null && jsonArray.size() > 0) {
                for (var apriltagJson : jsonArray) {
                    Apriltag apriltag = new Apriltag();
                    JsonObject apriltagObject = apriltagJson.getAsJsonObject();
                    apriltag.tagId = apriltagObject.get("tag_id").getAsString();
                    apriltag.position = new double[]{
                            parseJsonElementAsDouble(apriltagObject.get("position").getAsJsonArray().get(0)),
                            parseJsonElementAsDouble(apriltagObject.get("position").getAsJsonArray().get(1)),
                            parseJsonElementAsDouble(apriltagObject.get("position").getAsJsonArray().get(2))
                    };
                    apriltag.orientation = new double[]{
                            parseJsonElementAsDouble(apriltagObject.get("orientation").getAsJsonArray().get(0)),
                            parseJsonElementAsDouble(apriltagObject.get("orientation").getAsJsonArray().get(1)),
                            parseJsonElementAsDouble(apriltagObject.get("orientation").getAsJsonArray().get(2))
                    };
                    apriltag.distance = apriltagObject.get("distance").getAsDouble();
                    apriltag.horizontalAngle = apriltagObject.get("horizontal_angle").getAsDouble();
                    apriltag.verticalAngle = apriltagObject.get("vertical_angle").getAsDouble();
                    apriltags.add(apriltag);
                }
                apriltags.get(0).fullString = pMessage;
            }

            return apriltags;
        } catch (Exception e) {
            e.printStackTrace();
            return new ArrayList<>();
        }
    }
    public void disconnect() {
        if (webSocket != null) {
            webSocket.sendClose(WebSocket.NORMAL_CLOSURE, "Disconnecting")
                    .thenRun(() -> System.out.println("WebSocket closed"));
        }
    }
    private static class WebSocketListener implements Listener {
        private final CameraWebsocketClient client;

        public WebSocketListener(CameraWebsocketClient client) {
            this.client = client;
            startKeepAlivePing();
        }

        private void startKeepAlivePing() {
            new Thread(() -> {
                while (client.isConnected()) {
                    try {
                        client.sendMessage("ping");
                        Thread.sleep(10 * 1000);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
            }).start();
        }

        @Override
        public void onOpen(WebSocket webSocket) {
            System.out.println("WebSocket opened");
            Listener.super.onOpen(webSocket);
        }

        @Override
        public CompletionStage<?> onText(WebSocket webSocket, CharSequence data, boolean last) {
            client.onMessage(data.toString());
            return Listener.super.onText(webSocket, data, last);
        }

        @Override
        public void onError(WebSocket webSocket, Throwable error) {
            error.printStackTrace();
        }

        @Override
        public CompletionStage<?> onClose(WebSocket webSocket, int statusCode, String reason) {
            System.out.println("WebSocket closed with status " + statusCode + " and reason " + reason);
            if (statusCode != WebSocket.NORMAL_CLOSURE) {
                client.setupConnection();
            }
            return Listener.super.onClose(webSocket, statusCode, reason);
        }
    }

    // There are a lot more functions I can write but I think this is enough for the meeting. I can write more if you want.
}