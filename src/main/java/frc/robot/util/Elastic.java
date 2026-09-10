package frc.robot.util;

import io.avaje.jsonb.Json;
import io.avaje.jsonb.Jsonb;
import io.avaje.jsonb.JsonType;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.PubSubOption;
import org.wpilib.networktables.StringPublisher;
import org.wpilib.networktables.StringTopic;

public final class Elastic {
  private static final StringTopic notificationTopic =
      NetworkTableInstance.getDefault().getStringTopic("/Elastic/RobotNotifications");
  private static final StringPublisher notificationPublisher =
      notificationTopic.publish(new PubSubOption.SendAll(true), new PubSubOption.KeepDuplicates(true));
  private static final StringTopic selectedTabTopic =
      NetworkTableInstance.getDefault().getStringTopic("/Elastic/SelectedTab");
  private static final StringPublisher selectedTabPublisher =
      selectedTabTopic.publish(new PubSubOption.KeepDuplicates(true));

  // Build the lightweight Avaje context and generate a fast type pipeline for the Notification class
  private static final Jsonb jsonb = Jsonb.builder().build();
  private static final JsonType<Notification> notificationType = jsonb.type(Notification.class);

  /**
   * Represents the possible levels of notifications for the Elastic dashboard. These levels are
   * used to indicate the severity or type of notification.
   */
  public enum NotificationLevel {
    /** Informational Message */
    INFO,
    /** Warning message */
    WARNING,
    /** Error message */
    ERROR
  }

  /**
   * Sends an notification to the Elastic dashboard. The notification is serialized as a JSON string
   * before being published.
   *
   * @param notification the {@link Notification} object containing notification details
   */
  public static void sendNotification(Notification notification) {
    try {
      // Replaced Jackson writeValueAsString with Avaje compilation-safe toJson translation
      notificationPublisher.set(notificationType.toJson(notification));
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  /**
   * Selects the tab of the dashboard with the given name. If no tab matches the name, this will
   * have no effect on the widgets or tabs in view.
   *
   * <p>If the given name is a number, Elastic will select the tab whose index equals the number
   * provided.
   *
   * @param tabName the name of the tab to select
   */
  public static void selectTab(String tabName) {
    selectedTabPublisher.set(tabName);
  }

  /**
   * Selects the tab of the dashboard at the given index. If this index is greater than or equal to
   * the number of tabs, this will have no effect.
   *
   * @param tabIndex the index of the tab to select.
   */
  public static void selectTab(int tabIndex) {
    selectTab(Integer.toString(tabIndex));
  }

  /**
   * Represents an notification object to be sent to the Elastic dashboard. This object holds
   * properties such as level, title, description, display time, and dimensions to control how the
   * notification is displayed on the dashboard.
   */
  @Json // 👈 Required: Informs the Avaje processor to generate a companion serializer class at build time
  public static class Notification {
    
    // Jackson's @JsonProperty annotations are replaced with Avaje's standard alias rules or auto-matched field mappings
    private NotificationLevel level;
    private String title;
    private String description;
    
    @Json.Property("displayTime") // Explicitly map JSON camelCase key to this specific internal variable 
    private int displayTimeMillis;
    
    private double width;
    private double height;

    /**
     * Creates a new Notification with all default parameters. This constructor is intended to be
     * used with the chainable decorator methods
     *
     * <p>Title and description fields are empty.
     */
    public Notification() {
      this(NotificationLevel.INFO, "", "");
    }

    /**
     * Creates a new Notification with all properties specified.
     *
     * @param level the level of the notification (e.g., INFO, WARNING, ERROR)
     * @param title the title text of the notification
     * @param description the descriptive text of the notification
     * @param displayTimeMillis the time in milliseconds for which the notification is displayed
     * @param width the width of the notification display area
     * @param height the height of the notification display area, inferred if below zero
     */
    public Notification(
        NotificationLevel level,
        String title,
        String description,
        int displayTimeMillis,
        double width,
        double height) {
      this.level = level;
      this.title = title;
      this.displayTimeMillis = displayTimeMillis;
      this.description = description;
      this.height = height;
      this.width = width;
    }

    /**
     * Creates a new Notification with default display time and dimensions.
     *
     * @param level the level of the notification
     * @param title the title text of the notification
     * @param description the descriptive text of the notification
     */
    public Notification(NotificationLevel level, String title, String description) {
      this(level, title, description, 3000, 350, -1);
    }

    /**
     * Creates a new Notification with a specified display time and default dimensions.
     *
     * @param level the level of the notification
     * @param title the title text of the notification
     * @param description the descriptive text of the notification
     * @param displayTimeMillis the display time in milliseconds
     */
    public Notification(
        NotificationLevel level, String title, String description, int displayTimeMillis) {
      this(level, title, description, displayTimeMillis, 350, -1);
    }

    /**
     * Creates a new Notification with specified dimensions and default display time. If the height
     * is below zero, it is automatically inferred based on screen size.
     *
     * @param level the level of the notification
     * @param title the title text of the notification
     * @param description the descriptive text of the notification
     * @param width the width of the notification display area
     * @param height the height of the notification display area, inferred if below zero
     */
    public Notification(
        NotificationLevel level, String title, String description, double width, double height) {
      this(level, title, description, 3000, width, height);
    }

    /**
     * Updates the level of this notification
     *
     * @param level the level to set the notification to
     */
    public void setLevel(NotificationLevel level) {
      this.level = level;
    }

    /**
     * @return the level of this notification
     */
    public NotificationLevel getLevel() {
      return level;
    }

    /**
     * Updates the title of this notification
     *
     * @param title the title to set the notification to
     */
    public void setTitle(String title) {
      this.title = title;
    }

    /**
     * Gets the title of this notification
     *
     * @return the title of this notification
     */
    public String getTitle() {
      return title;
    }

    /**
     * Updates the description of this notification
     *
     * @param description the description to set the notification to
     */
    public void setDescription(String description) {
      this.description = description;
    }

    public String getDescription() {
      return description;
    }

    /**
     * Updates the display time of the notification
     *
     * @param seconds the number of seconds to display the notification for
     */
    public void setDisplayTimeSeconds(double seconds) {
      setDisplayTimeMillis((int) Math.round(seconds * 1000));
    }

    /**
     * Updates the display time of the notification in milliseconds
     *
     * @param displayTimeMillis the number of milliseconds to display the notification for
     */
    public void setDisplayTimeMillis(int displayTimeMillis) {
      this.displayTimeMillis = displayTimeMillis;
    }

    /**
     * Gets the display time of the notification in milliseconds
     *
     * @return the number of milliseconds the notification is displayed for
     */
    public int getDisplayTimeMillis() {
      return displayTimeMillis;
    }

    public double getWidth() {
      return width;
    }

    public void setWidth(double width) {
      this.width = width;
    }

    public double getHeight() {
      return height;
    }

    public void setHeight(double height) {
      this.height = height;
    }
  }
}