package io.swagger.model;

import java.util.Objects;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.annotation.JsonCreator;
import io.swagger.v3.oas.annotations.media.Schema;
import org.springframework.validation.annotation.Validated;
import javax.validation.Valid;
import javax.validation.constraints.*;

/**
 * UpdateVoiceAssistantChat
 */
@Validated
@javax.annotation.Generated(value = "io.swagger.codegen.v3.generators.java.SpringCodegen", date = "2023-10-02T10:03:56.193162043Z[GMT]")


public class UpdateVoiceAssistantChat   {
  @JsonProperty("title")
  private String title = null;

  @JsonProperty("personality")
  private String personality = null;

  public UpdateVoiceAssistantChat title(String title) {
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

  public UpdateVoiceAssistantChat personality(String personality) {
    this.personality = personality;
    return this;
  }

  /**
   * Get personality
   * @return personality
   **/
  @Schema(example = "Janina", required = true, description = "")
      @NotNull

    public String getPersonality() {
    return personality;
  }

  public void setPersonality(String personality) {
    this.personality = personality;
  }


  @Override
  public boolean equals(java.lang.Object o) {
    if (this == o) {
      return true;
    }
    if (o == null || getClass() != o.getClass()) {
      return false;
    }
    UpdateVoiceAssistantChat updateVoiceAssistantChat = (UpdateVoiceAssistantChat) o;
    return Objects.equals(this.title, updateVoiceAssistantChat.title) &&
        Objects.equals(this.personality, updateVoiceAssistantChat.personality);
  }

  @Override
  public int hashCode() {
    return Objects.hash(title, personality);
  }

  @Override
  public String toString() {
    StringBuilder sb = new StringBuilder();
    sb.append("class UpdateVoiceAssistantChat {\n");
    
    sb.append("    title: ").append(toIndentedString(title)).append("\n");
    sb.append("    personality: ").append(toIndentedString(personality)).append("\n");
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
