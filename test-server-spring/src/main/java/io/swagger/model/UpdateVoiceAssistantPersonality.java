package io.swagger.model;

import java.util.Objects;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.annotation.JsonCreator;
import io.swagger.v3.oas.annotations.media.Schema;
import java.math.BigDecimal;
import org.springframework.validation.annotation.Validated;
import javax.validation.Valid;
import javax.validation.constraints.*;

/**
 * UpdateVoiceAssistantPersonality
 */
@Validated
@javax.annotation.Generated(value = "io.swagger.codegen.v3.generators.java.SpringCodegen", date = "2023-09-27T09:57:56.878396287Z[GMT]")


public class UpdateVoiceAssistantPersonality   {
  @JsonProperty("id")
  private String id = null;

  @JsonProperty("name")
  private String name = null;

  @JsonProperty("description")
  private String description = null;

  @JsonProperty("gender")
  private String gender = null;

  @JsonProperty("pauseThreshold")
  private BigDecimal pauseThreshold = null;

  public UpdateVoiceAssistantPersonality id(String id) {
    this.id = id;
    return this;
  }

  /**
   * Get id
   * @return id
   **/
  @Schema(example = "UUID", description = "")
  
    public String getId() {
    return id;
  }

  public void setId(String id) {
    this.id = id;
  }

  public UpdateVoiceAssistantPersonality name(String name) {
    this.name = name;
    return this;
  }

  /**
   * Get name
   * @return name
   **/
  @Schema(example = "Name of the voice assistant personality", description = "")
  
    public String getName() {
    return name;
  }

  public void setName(String name) {
    this.name = name;
  }

  public UpdateVoiceAssistantPersonality description(String description) {
    this.description = description;
    return this;
  }

  /**
   * Get description
   * @return description
   **/
  @Schema(example = "Personality Description", description = "")
  
    public String getDescription() {
    return description;
  }

  public void setDescription(String description) {
    this.description = description;
  }

  public UpdateVoiceAssistantPersonality gender(String gender) {
    this.gender = gender;
    return this;
  }

  /**
   * Get gender
   * @return gender
   **/
  @Schema(example = "Male/Female", description = "")
  
    public String getGender() {
    return gender;
  }

  public void setGender(String gender) {
    this.gender = gender;
  }

  public UpdateVoiceAssistantPersonality pauseThreshold(BigDecimal pauseThreshold) {
    this.pauseThreshold = pauseThreshold;
    return this;
  }

  /**
   * Get pauseThreshold
   * @return pauseThreshold
   **/
  @Schema(example = "0.8", description = "")
  
    @Valid
    public BigDecimal getPauseThreshold() {
    return pauseThreshold;
  }

  public void setPauseThreshold(BigDecimal pauseThreshold) {
    this.pauseThreshold = pauseThreshold;
  }


  @Override
  public boolean equals(java.lang.Object o) {
    if (this == o) {
      return true;
    }
    if (o == null || getClass() != o.getClass()) {
      return false;
    }
    UpdateVoiceAssistantPersonality updateVoiceAssistantPersonality = (UpdateVoiceAssistantPersonality) o;
    return Objects.equals(this.id, updateVoiceAssistantPersonality.id) &&
        Objects.equals(this.name, updateVoiceAssistantPersonality.name) &&
        Objects.equals(this.description, updateVoiceAssistantPersonality.description) &&
        Objects.equals(this.gender, updateVoiceAssistantPersonality.gender) &&
        Objects.equals(this.pauseThreshold, updateVoiceAssistantPersonality.pauseThreshold);
  }

  @Override
  public int hashCode() {
    return Objects.hash(id, name, description, gender, pauseThreshold);
  }

  @Override
  public String toString() {
    StringBuilder sb = new StringBuilder();
    sb.append("class UpdateVoiceAssistantPersonality {\n");
    
    sb.append("    id: ").append(toIndentedString(id)).append("\n");
    sb.append("    name: ").append(toIndentedString(name)).append("\n");
    sb.append("    description: ").append(toIndentedString(description)).append("\n");
    sb.append("    gender: ").append(toIndentedString(gender)).append("\n");
    sb.append("    pauseThreshold: ").append(toIndentedString(pauseThreshold)).append("\n");
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
