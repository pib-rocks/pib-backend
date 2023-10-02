package io.swagger.model;

import java.util.Objects;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.annotation.JsonCreator;
import io.swagger.model.GetAllPersonalitiesVoiceAssistantPersonalities;
import io.swagger.v3.oas.annotations.media.Schema;
import java.util.ArrayList;
import java.util.List;
import org.springframework.validation.annotation.Validated;
import javax.validation.Valid;
import javax.validation.constraints.*;

/**
 * GetAllPersonalities
 */
@Validated
@javax.annotation.Generated(value = "io.swagger.codegen.v3.generators.java.SpringCodegen", date = "2023-10-02T10:03:56.193162043Z[GMT]")


public class GetAllPersonalities   {
  @JsonProperty("voiceAssistantPersonalities")
  @Valid
  private List<GetAllPersonalitiesVoiceAssistantPersonalities> voiceAssistantPersonalities = null;

  public GetAllPersonalities voiceAssistantPersonalities(List<GetAllPersonalitiesVoiceAssistantPersonalities> voiceAssistantPersonalities) {
    this.voiceAssistantPersonalities = voiceAssistantPersonalities;
    return this;
  }

  public GetAllPersonalities addVoiceAssistantPersonalitiesItem(GetAllPersonalitiesVoiceAssistantPersonalities voiceAssistantPersonalitiesItem) {
    if (this.voiceAssistantPersonalities == null) {
      this.voiceAssistantPersonalities = new ArrayList<GetAllPersonalitiesVoiceAssistantPersonalities>();
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
    public List<GetAllPersonalitiesVoiceAssistantPersonalities> getVoiceAssistantPersonalities() {
    return voiceAssistantPersonalities;
  }

  public void setVoiceAssistantPersonalities(List<GetAllPersonalitiesVoiceAssistantPersonalities> voiceAssistantPersonalities) {
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
    GetAllPersonalities getAllPersonalities = (GetAllPersonalities) o;
    return Objects.equals(this.voiceAssistantPersonalities, getAllPersonalities.voiceAssistantPersonalities);
  }

  @Override
  public int hashCode() {
    return Objects.hash(voiceAssistantPersonalities);
  }

  @Override
  public String toString() {
    StringBuilder sb = new StringBuilder();
    sb.append("class GetAllPersonalities {\n");
    
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
