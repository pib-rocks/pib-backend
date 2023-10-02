package io.swagger.model;

import java.util.Objects;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.annotation.JsonCreator;
import io.swagger.v3.oas.annotations.media.Schema;
import org.springframework.validation.annotation.Validated;
import javax.validation.Valid;
import javax.validation.constraints.*;

/**
 * AddVoiceAssistantChatMessageRequest
 */
@Validated
@javax.annotation.Generated(value = "io.swagger.codegen.v3.generators.java.SpringCodegen", date = "2023-10-02T10:03:56.193162043Z[GMT]")


public class AddVoiceAssistantChatMessageRequest   {
  @JsonProperty("message")
  private String message = null;

  public AddVoiceAssistantChatMessageRequest message(String message) {
    this.message = message;
    return this;
  }

  /**
   * Get message
   * @return message
   **/
  @Schema(example = "Hallo Thomas, erzähle mir etwas über Nürnberg.", description = "")
  
    public String getMessage() {
    return message;
  }

  public void setMessage(String message) {
    this.message = message;
  }


  @Override
  public boolean equals(java.lang.Object o) {
    if (this == o) {
      return true;
    }
    if (o == null || getClass() != o.getClass()) {
      return false;
    }
    AddVoiceAssistantChatMessageRequest addVoiceAssistantChatMessageRequest = (AddVoiceAssistantChatMessageRequest) o;
    return Objects.equals(this.message, addVoiceAssistantChatMessageRequest.message);
  }

  @Override
  public int hashCode() {
    return Objects.hash(message);
  }

  @Override
  public String toString() {
    StringBuilder sb = new StringBuilder();
    sb.append("class AddVoiceAssistantChatMessageRequest {\n");
    
    sb.append("    message: ").append(toIndentedString(message)).append("\n");
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
