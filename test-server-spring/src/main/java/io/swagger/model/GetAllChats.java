package io.swagger.model;

import java.util.Objects;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.annotation.JsonCreator;
import io.swagger.model.GetAllChatsVoiceAssistantChats;
import io.swagger.v3.oas.annotations.media.Schema;
import java.util.ArrayList;
import java.util.List;
import org.springframework.validation.annotation.Validated;
import javax.validation.Valid;
import javax.validation.constraints.*;

/**
 * GetAllChats
 */
@Validated
@javax.annotation.Generated(value = "io.swagger.codegen.v3.generators.java.SpringCodegen", date = "2023-10-02T10:03:56.193162043Z[GMT]")


public class GetAllChats   {
  @JsonProperty("voiceAssistantChats")
  @Valid
  private List<GetAllChatsVoiceAssistantChats> voiceAssistantChats = null;

  public GetAllChats voiceAssistantChats(List<GetAllChatsVoiceAssistantChats> voiceAssistantChats) {
    this.voiceAssistantChats = voiceAssistantChats;
    return this;
  }

  public GetAllChats addVoiceAssistantChatsItem(GetAllChatsVoiceAssistantChats voiceAssistantChatsItem) {
    if (this.voiceAssistantChats == null) {
      this.voiceAssistantChats = new ArrayList<GetAllChatsVoiceAssistantChats>();
    }
    this.voiceAssistantChats.add(voiceAssistantChatsItem);
    return this;
  }

  /**
   * Get voiceAssistantChats
   * @return voiceAssistantChats
   **/
  @Schema(description = "")
      @Valid
    public List<GetAllChatsVoiceAssistantChats> getVoiceAssistantChats() {
    return voiceAssistantChats;
  }

  public void setVoiceAssistantChats(List<GetAllChatsVoiceAssistantChats> voiceAssistantChats) {
    this.voiceAssistantChats = voiceAssistantChats;
  }


  @Override
  public boolean equals(java.lang.Object o) {
    if (this == o) {
      return true;
    }
    if (o == null || getClass() != o.getClass()) {
      return false;
    }
    GetAllChats getAllChats = (GetAllChats) o;
    return Objects.equals(this.voiceAssistantChats, getAllChats.voiceAssistantChats);
  }

  @Override
  public int hashCode() {
    return Objects.hash(voiceAssistantChats);
  }

  @Override
  public String toString() {
    StringBuilder sb = new StringBuilder();
    sb.append("class GetAllChats {\n");
    
    sb.append("    voiceAssistantChats: ").append(toIndentedString(voiceAssistantChats)).append("\n");
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
