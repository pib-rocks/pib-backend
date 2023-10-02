package io.swagger.model;

import java.util.Objects;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.annotation.JsonCreator;
import io.swagger.model.AddVoiceAssistantChatMessageResponseAiResponse;
import io.swagger.model.GetVoiceAssistantChatMessagesMessages;
import io.swagger.v3.oas.annotations.media.Schema;
import org.springframework.validation.annotation.Validated;
import javax.validation.Valid;
import javax.validation.constraints.*;

/**
 * AddVoiceAssistantChatMessageResponse
 */
@Validated
@javax.annotation.Generated(value = "io.swagger.codegen.v3.generators.java.SpringCodegen", date = "2023-10-02T10:03:56.193162043Z[GMT]")


public class AddVoiceAssistantChatMessageResponse   {
  @JsonProperty("user_prompt")
  private GetVoiceAssistantChatMessagesMessages userPrompt = null;

  @JsonProperty("ai_response")
  private AddVoiceAssistantChatMessageResponseAiResponse aiResponse = null;

  public AddVoiceAssistantChatMessageResponse userPrompt(GetVoiceAssistantChatMessagesMessages userPrompt) {
    this.userPrompt = userPrompt;
    return this;
  }

  /**
   * Get userPrompt
   * @return userPrompt
   **/
  @Schema(description = "")
  
    @Valid
    public GetVoiceAssistantChatMessagesMessages getUserPrompt() {
    return userPrompt;
  }

  public void setUserPrompt(GetVoiceAssistantChatMessagesMessages userPrompt) {
    this.userPrompt = userPrompt;
  }

  public AddVoiceAssistantChatMessageResponse aiResponse(AddVoiceAssistantChatMessageResponseAiResponse aiResponse) {
    this.aiResponse = aiResponse;
    return this;
  }

  /**
   * Get aiResponse
   * @return aiResponse
   **/
  @Schema(description = "")
  
    @Valid
    public AddVoiceAssistantChatMessageResponseAiResponse getAiResponse() {
    return aiResponse;
  }

  public void setAiResponse(AddVoiceAssistantChatMessageResponseAiResponse aiResponse) {
    this.aiResponse = aiResponse;
  }


  @Override
  public boolean equals(java.lang.Object o) {
    if (this == o) {
      return true;
    }
    if (o == null || getClass() != o.getClass()) {
      return false;
    }
    AddVoiceAssistantChatMessageResponse addVoiceAssistantChatMessageResponse = (AddVoiceAssistantChatMessageResponse) o;
    return Objects.equals(this.userPrompt, addVoiceAssistantChatMessageResponse.userPrompt) &&
        Objects.equals(this.aiResponse, addVoiceAssistantChatMessageResponse.aiResponse);
  }

  @Override
  public int hashCode() {
    return Objects.hash(userPrompt, aiResponse);
  }

  @Override
  public String toString() {
    StringBuilder sb = new StringBuilder();
    sb.append("class AddVoiceAssistantChatMessageResponse {\n");
    
    sb.append("    userPrompt: ").append(toIndentedString(userPrompt)).append("\n");
    sb.append("    aiResponse: ").append(toIndentedString(aiResponse)).append("\n");
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
