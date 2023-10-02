package io.swagger.model;

import java.util.Objects;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.annotation.JsonCreator;
import io.swagger.model.GetVoiceAssistantChatMessagesMessages;
import io.swagger.v3.oas.annotations.media.Schema;
import java.util.ArrayList;
import java.util.List;
import org.springframework.validation.annotation.Validated;
import javax.validation.Valid;
import javax.validation.constraints.*;

/**
 * GetVoiceAssistantChatMessages
 */
@Validated
@javax.annotation.Generated(value = "io.swagger.codegen.v3.generators.java.SpringCodegen", date = "2023-10-02T10:03:56.193162043Z[GMT]")


public class GetVoiceAssistantChatMessages   {
  @JsonProperty("messages")
  @Valid
  private List<GetVoiceAssistantChatMessagesMessages> messages = null;

  public GetVoiceAssistantChatMessages messages(List<GetVoiceAssistantChatMessagesMessages> messages) {
    this.messages = messages;
    return this;
  }

  public GetVoiceAssistantChatMessages addMessagesItem(GetVoiceAssistantChatMessagesMessages messagesItem) {
    if (this.messages == null) {
      this.messages = new ArrayList<GetVoiceAssistantChatMessagesMessages>();
    }
    this.messages.add(messagesItem);
    return this;
  }

  /**
   * Get messages
   * @return messages
   **/
  @Schema(description = "")
      @Valid
    public List<GetVoiceAssistantChatMessagesMessages> getMessages() {
    return messages;
  }

  public void setMessages(List<GetVoiceAssistantChatMessagesMessages> messages) {
    this.messages = messages;
  }


  @Override
  public boolean equals(java.lang.Object o) {
    if (this == o) {
      return true;
    }
    if (o == null || getClass() != o.getClass()) {
      return false;
    }
    GetVoiceAssistantChatMessages getVoiceAssistantChatMessages = (GetVoiceAssistantChatMessages) o;
    return Objects.equals(this.messages, getVoiceAssistantChatMessages.messages);
  }

  @Override
  public int hashCode() {
    return Objects.hash(messages);
  }

  @Override
  public String toString() {
    StringBuilder sb = new StringBuilder();
    sb.append("class GetVoiceAssistantChatMessages {\n");
    
    sb.append("    messages: ").append(toIndentedString(messages)).append("\n");
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
