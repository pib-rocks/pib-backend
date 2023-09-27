package io.swagger.model;

import java.util.Objects;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.annotation.JsonCreator;
import io.swagger.model.GetPersonalitiesVoiceAssistantPersonalities;
import io.swagger.v3.oas.annotations.media.Schema;
import java.util.ArrayList;
import java.util.List;
import org.springframework.validation.annotation.Validated;
import javax.validation.Valid;
import javax.validation.constraints.*;

/**
 * GetPersonalities
 */
@Validated
@javax.annotation.Generated(value = "io.swagger.codegen.v3.generators.java.SpringCodegen", date = "2023-09-27T09:57:56.878396287Z[GMT]")


public class GetPersonalities   {
  @JsonProperty("voiceAssistantPersonalities")
  @Valid
  private List<GetPersonalitiesVoiceAssistantPersonalities> voiceAssistantPersonalities = null;

  public GetPersonalities voiceAssistantPersonalities(List<GetPersonalitiesVoiceAssistantPersonalities> voiceAssistantPersonalities) {
    this.voiceAssistantPersonalities = voiceAssistantPersonalities;
    return this;
  }

  public GetPersonalities addVoiceAssistantPersonalitiesItem(GetPersonalitiesVoiceAssistantPersonalities voiceAssistantPersonalitiesItem) {
    if (this.voiceAssistantPersonalities == null) {
      this.voiceAssistantPersonalities = new ArrayList<GetPersonalitiesVoiceAssistantPersonalities>();
    }
    this.voiceAssistantPersonalities.add(voiceAssistantPersonalitiesItem);
    return this;
  }

  /**
   * Get voiceAssistantPersonalities
   * @return voiceAssistantPersonalities
   **/
  @Schema(description = "")
      @Valid
    public List<GetPersonalitiesVoiceAssistantPersonalities> getVoiceAssistantPersonalities() {
    return voiceAssistantPersonalities;
  }

  public void setVoiceAssistantPersonalities(List<GetPersonalitiesVoiceAssistantPersonalities> voiceAssistantPersonalities) {
    this.voiceAssistantPersonalities = voiceAssistantPersonalities;
  }


  @Override
  public boolean equals(java.lang.Object o) {
    if (this == o) {
      return true;
    }
    if (o == null || getClass() != o.getClass()) {
      return false;
    }
    GetPersonalities getPersonalities = (GetPersonalities) o;
    return Objects.equals(this.voiceAssistantPersonalities, getPersonalities.voiceAssistantPersonalities);
  }

  @Override
  public int hashCode() {
    return Objects.hash(voiceAssistantPersonalities);
  }

  @Override
  public String toString() {
    StringBuilder sb = new StringBuilder();
    sb.append("class GetPersonalities {\n");
    
    sb.append("    voiceAssistantPersonalities: ").append(toIndentedString(voiceAssistantPersonalities)).append("\n");
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
