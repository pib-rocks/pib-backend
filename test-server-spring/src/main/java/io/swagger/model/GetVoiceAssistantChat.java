package io.swagger.model;

import java.util.Objects;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.annotation.JsonCreator;
import io.swagger.v3.oas.annotations.media.Schema;
import org.springframework.validation.annotation.Validated;
import javax.validation.Valid;
import javax.validation.constraints.*;

/**
 * GetVoiceAssistantChat
 */
@Validated
@javax.annotation.Generated(value = "io.swagger.codegen.v3.generators.java.SpringCodegen", date = "2023-10-02T10:03:56.193162043Z[GMT]")


public class GetVoiceAssistantChat   {
  @JsonProperty("title")
  private String title = null;

  @JsonProperty("personality_id")
  private String personalityId = null;

  @JsonProperty("personality_name")
  private String personalityName = null;

  public GetVoiceAssistantChat title(String title) {
    this.title = title;
    return this;
  }

  /**
   * Get title
   * @return title
   **/
  @Schema(example = "Short Stories", description = "")
  
    public String getTitle() {
    return title;
  }

  public void setTitle(String title) {
    this.title = title;
  }

  public GetVoiceAssistantChat personalityId(String personalityId) {
    this.personalityId = personalityId;
    return this;
  }

  /**
   * Get personalityId
   * @return personalityId
   **/
  @Schema(example = "UUID", description = "")
  
    public String getPersonalityId() {
    return personalityId;
  }

  public void setPersonalityId(String personalityId) {
    this.personalityId = personalityId;
  }

  public GetVoiceAssistantChat personalityName(String personalityName) {
    this.personalityName = personalityName;
    return this;
  }

  /**
   * Get personalityName
   * @return personalityName
   **/
  @Schema(example = "Thomas", description = "")
  
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
    GetVoiceAssistantChat getVoiceAssistantChat = (GetVoiceAssistantChat) o;
    return Objects.equals(this.title, getVoiceAssistantChat.title) &&
        Objects.equals(this.personalityId, getVoiceAssistantChat.personalityId) &&
        Objects.equals(this.personalityName, getVoiceAssistantChat.personalityName);
  }

  @Override
  public int hashCode() {
    return Objects.hash(title, personalityId, personalityName);
  }

  @Override
  public String toString() {
    StringBuilder sb = new StringBuilder();
    sb.append("class GetVoiceAssistantChat {\n");
    
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
