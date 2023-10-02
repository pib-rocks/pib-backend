package io.swagger.model;

import java.util.Objects;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.annotation.JsonCreator;
import io.swagger.v3.oas.annotations.media.Schema;
import org.springframework.validation.annotation.Validated;
import javax.validation.Valid;
import javax.validation.constraints.*;

/**
 * SuccessfulVoiceAssistantChatCreationResponse
 */
@Validated
@javax.annotation.Generated(value = "io.swagger.codegen.v3.generators.java.SpringCodegen", date = "2023-10-02T10:03:56.193162043Z[GMT]")


public class SuccessfulVoiceAssistantChatCreationResponse   {
  @JsonProperty("chat_id")
  private String chatId = null;

  @JsonProperty("title")
  private String title = null;

  @JsonProperty("personality_id")
  private String personalityId = null;

  @JsonProperty("personality_name")
  private String personalityName = null;

  public SuccessfulVoiceAssistantChatCreationResponse chatId(String chatId) {
    this.chatId = chatId;
    return this;
  }

  /**
   * Get chatId
   * @return chatId
   **/
  @Schema(example = "UUID", required = true, description = "")
      @NotNull

    public String getChatId() {
    return chatId;
  }

  public void setChatId(String chatId) {
    this.chatId = chatId;
  }

  public SuccessfulVoiceAssistantChatCreationResponse title(String title) {
    this.title = title;
    return this;
  }

  /**
   * Get title
   * @return title
   **/
  @Schema(example = "Short Stories", required = true, description = "")
      @NotNull

    public String getTitle() {
    return title;
  }

  public void setTitle(String title) {
    this.title = title;
  }

  public SuccessfulVoiceAssistantChatCreationResponse personalityId(String personalityId) {
    this.personalityId = personalityId;
    return this;
  }

  /**
   * Get personalityId
   * @return personalityId
   **/
  @Schema(example = "UUID", required = true, description = "")
      @NotNull

    public String getPersonalityId() {
    return personalityId;
  }

  public void setPersonalityId(String personalityId) {
    this.personalityId = personalityId;
  }

  public SuccessfulVoiceAssistantChatCreationResponse personalityName(String personalityName) {
    this.personalityName = personalityName;
    return this;
  }

  /**
   * Get personalityName
   * @return personalityName
   **/
  @Schema(example = "Janina", required = true, description = "")
      @NotNull

    public String getPersonalityName() {
    return personalityName;
  }

  public void setPersonalityName(String personalityName) {
    this.personalityName = personalityName;
  }


  @Override
  public boolean equals(java.lang.Object o) {
    if (this == o) {
      return true;
    }
    if (o == null || getClass() != o.getClass()) {
      return false;
    }
    SuccessfulVoiceAssistantChatCreationResponse successfulVoiceAssistantChatCreationResponse = (SuccessfulVoiceAssistantChatCreationResponse) o;
    return Objects.equals(this.chatId, successfulVoiceAssistantChatCreationResponse.chatId) &&
        Objects.equals(this.title, successfulVoiceAssistantChatCreationResponse.title) &&
        Objects.equals(this.personalityId, successfulVoiceAssistantChatCreationResponse.personalityId) &&
        Objects.equals(this.personalityName, successfulVoiceAssistantChatCreationResponse.personalityName);
  }

  @Override
  public int hashCode() {
    return Objects.hash(chatId, title, personalityId, personalityName);
  }

  @Override
  public String toString() {
    StringBuilder sb = new StringBuilder();
    sb.append("class SuccessfulVoiceAssistantChatCreationResponse {\n");
    
    sb.append("    chatId: ").append(toIndentedString(chatId)).append("\n");
    sb.append("    title: ").append(toIndentedString(title)).append("\n");
    sb.append("    personalityId: ").append(toIndentedString(personalityId)).append("\n");
    sb.append("    personalityName: ").append(toIndentedString(personalityName)).append("\n");
    sb.append("}");
    return sb.toString();
  }

  /**
   * Convert the given object to string with each line indented by 4 spaces
   * (except the first line).
   */
  private String toIndentedString(java.lang.Object o) {
    if (o == null) {
      return "null";
    }
    return o.toString().replace("\n", "\n    ");
  }
}
